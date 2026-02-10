//! Search engine for exhaustive tree search over placement layers.
//!
//! The `SearchEngine` drives a depth-first search over the layer stack,
//! generating candidates lazily (one at a time) to keep memory proportional
//! to tree depth rather than tree width. It supports CPU budgeting for
//! incremental multi-tick execution and serialization for cross-tick persistence.

use crate::layer::*;
use crate::pipeline::CpuBudget;
use crate::terrain::*;
use log::*;
use serde::{Deserialize, Serialize};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

/// A single frame in the search stack, representing the state at one layer depth.
#[derive(Clone, Serialize, Deserialize)]
pub struct SearchFrame {
    /// Index of the layer in the layer stack.
    pub layer_index: usize,
    /// The PlacementState entering this layer (before any candidate is applied).
    pub input_state: PlacementState,
    /// Index of the next candidate to request from the layer.
    pub next_candidate_index: usize,
}

/// Result of a single step of the search engine.
pub enum SearchResult {
    /// Search is still in progress (CPU budget exhausted).
    Running,
    /// Search is complete. The best plan is stored in the engine.
    Complete,
}

/// Drives the exhaustive search over the layer stack.
#[derive(Serialize, Deserialize)]
pub struct SearchEngine {
    /// The ordered layer stack (not serialized -- provided by caller each tick).
    #[serde(skip)]
    layers: Option<Vec<Box<dyn PlacementLayer>>>,
    /// Search state: stack of frames (one per layer depth).
    stack: Vec<SearchFrame>,
    /// Best complete plan found so far (and its score).
    best: Option<(PlacementState, f32)>,
    /// Best complete score found so far (for pruning).
    best_complete_score: f32,
    /// Pruning margin: skip candidates whose cumulative score is more
    /// than this amount below best_complete_score.
    prune_margin: f32,
    /// Fingerprint of the layer stack (hash of layer names).
    layer_fingerprint: u64,
    /// Total candidates evaluated (for progress reporting).
    candidates_evaluated: u64,
    /// Total candidates pruned (for progress reporting).
    candidates_pruned: u64,
    /// Total complete plans found (for progress reporting).
    complete_plans_found: u64,
    /// Per-layer rejection counts (for debugging).
    #[serde(skip)]
    layer_rejections: Vec<u64>,
    /// Per-layer exhaustion counts (for debugging).
    #[serde(skip)]
    layer_exhaustions: Vec<u64>,
}

// Manual Serialize impl for Box<dyn PlacementLayer> is not needed since it's #[serde(skip)].
// We need a dummy impl for the trait object to satisfy the compiler when used in Option.
impl Serialize for Box<dyn PlacementLayer> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        // This should never be called due to #[serde(skip)]
        serializer.serialize_unit()
    }
}

impl<'de> Deserialize<'de> for Box<dyn PlacementLayer> {
    fn deserialize<D>(_deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Err(serde::de::Error::custom(
            "PlacementLayer trait objects cannot be deserialized directly",
        ))
    }
}

impl SearchEngine {
    /// Create a new search engine with the given layers, initial state, and pruning margin.
    pub fn new(
        layers: Vec<Box<dyn PlacementLayer>>,
        initial_state: PlacementState,
        prune_margin: f32,
    ) -> Self {
        let fingerprint = compute_fingerprint(&layers);

        let num_layers = layers.len();
        let mut engine = SearchEngine {
            layers: Some(layers),
            stack: Vec::new(),
            best: None,
            best_complete_score: f32::NEG_INFINITY,
            prune_margin,
            layer_fingerprint: fingerprint,
            candidates_evaluated: 0,
            candidates_pruned: 0,
            complete_plans_found: 0,
            layer_rejections: vec![0; num_layers],
            layer_exhaustions: vec![0; num_layers],
        };

        // Push the initial frame for the first applicable layer
        engine.push_initial_frame(initial_state);
        engine
    }

    /// Inject layers into a deserialized engine. Returns false if fingerprint mismatches.
    pub fn inject_layers(&mut self, layers: Vec<Box<dyn PlacementLayer>>) -> bool {
        let fingerprint = compute_fingerprint(&layers);
        if fingerprint != self.layer_fingerprint {
            return false;
        }
        self.layers = Some(layers);
        true
    }

    /// Get the fingerprint of the layer stack.
    pub fn fingerprint(&self) -> u64 {
        self.layer_fingerprint
    }

    /// Get the best plan found so far, if any.
    pub fn best_plan(&self) -> Option<&PlacementState> {
        self.best.as_ref().map(|(state, _)| state)
    }

    /// Get the best score found so far.
    pub fn best_score(&self) -> f32 {
        self.best_complete_score
    }

    /// Take the best plan out of the engine (consuming it).
    pub fn take_best_plan(self) -> Option<PlacementState> {
        self.best.map(|(state, _)| state)
    }

    /// Get progress statistics.
    pub fn stats(&self) -> SearchStats {
        SearchStats {
            candidates_evaluated: self.candidates_evaluated,
            candidates_pruned: self.candidates_pruned,
            complete_plans_found: self.complete_plans_found,
            best_score: self.best_complete_score,
            stack_depth: self.stack.len(),
        }
    }

    /// Run the search until CPU budget is exhausted or search is complete.
    pub fn step(&mut self, terrain: &FastRoomTerrain, budget: &CpuBudget) -> SearchResult {
        let layers = self
            .layers
            .as_ref()
            .expect("layers must be injected before stepping");

        loop {
            // If the stack is empty, the search is complete
            if self.stack.is_empty() {
                debug!(
                    "Search complete: evaluated={}, pruned={}, complete_plans={}",
                    self.candidates_evaluated,
                    self.candidates_pruned,
                    self.complete_plans_found
                );
                // Log per-layer stats
                for (i, (rej, exh)) in self
                    .layer_rejections
                    .iter()
                    .zip(self.layer_exhaustions.iter())
                    .enumerate()
                {
                    if *rej > 0 || *exh > 0 {
                        let name = layers
                            .get(i)
                            .map(|l| l.name())
                            .unwrap_or("?");
                        debug!(
                            "  Layer {} '{}': rejections={}, exhaustions={}",
                            i, name, rej, exh
                        );
                    }
                }
                return SearchResult::Complete;
            }

            let frame_idx = self.stack.len() - 1;
            let frame = &self.stack[frame_idx];
            let layer_index = frame.layer_index;
            let candidate_index = frame.next_candidate_index;
            let input_state = frame.input_state.clone();

            let layer = &layers[layer_index];

            // Generate the next candidate
            let candidate_result = layer.candidate(candidate_index, &input_state, terrain);

            // Increment the candidate index for this frame
            self.stack[frame_idx].next_candidate_index += 1;

            match candidate_result {
                None => {
                    // This layer is exhausted. Pop the frame.
                    if layer_index < self.layer_exhaustions.len() {
                        self.layer_exhaustions[layer_index] += 1;
                    }
                    self.stack.pop();
                    // The parent frame's next_candidate_index was already incremented
                    // when we pushed this frame, so we just continue.
                }
                Some(Err(())) => {
                    // Candidate rejected as structurally invalid. Skip it.
                    self.candidates_evaluated += 1;
                    if layer_index < self.layer_rejections.len() {
                        self.layer_rejections[layer_index] += 1;
                    }
                    trace!(
                        "Layer '{}' (index {}) rejected candidate {}",
                        layer.name(),
                        layer_index,
                        candidate_index
                    );
                }
                Some(Ok(candidate_state)) => {
                    self.candidates_evaluated += 1;

                    // Score pruning
                    let cumulative = candidate_state.cumulative_score();
                    if self.best_complete_score > f32::NEG_INFINITY
                        && cumulative + self.prune_margin < self.best_complete_score
                    {
                        self.candidates_pruned += 1;
                        // Skip this candidate -- score too low
                    } else {
                        // Find the next applicable layer
                        let next_layer = self.find_next_applicable_layer(
                            layer_index + 1,
                            &candidate_state,
                            layers,
                        );

                        match next_layer {
                            Some(next_layer_index) => {
                                // Push a new frame for the next layer
                                self.stack.push(SearchFrame {
                                    layer_index: next_layer_index,
                                    input_state: candidate_state,
                                    next_candidate_index: 0,
                                });
                            }
                            None => {
                                // This is a complete plan (all layers exhausted)
                                self.complete_plans_found += 1;
                                if cumulative > self.best_complete_score {
                                    self.best_complete_score = cumulative;
                                    self.best = Some((candidate_state, cumulative));
                                }
                            }
                        }
                    }
                }
            }

            // Check CPU budget
            if !budget.has_budget() {
                return SearchResult::Running;
            }
        }
    }

    /// Find the next applicable layer starting from `start_index`.
    /// Layers that are not applicable are skipped (state passes through unchanged).
    fn find_next_applicable_layer(
        &self,
        start_index: usize,
        state: &PlacementState,
        layers: &[Box<dyn PlacementLayer>],
    ) -> Option<usize> {
        (start_index..layers.len()).find(|&i| layers[i].is_applicable(state))
    }

    /// Push the initial frame, finding the first applicable layer.
    fn push_initial_frame(&mut self, initial_state: PlacementState) {
        let layers = self.layers.as_ref().expect("layers must be set");
        if let Some(first_layer) = self.find_next_applicable_layer(0, &initial_state, layers) {
            self.stack.push(SearchFrame {
                layer_index: first_layer,
                input_state: initial_state,
                next_candidate_index: 0,
            });
        }
        // If no layers are applicable, the search will immediately complete
        // with no best plan.
    }
}

/// Progress statistics for the search.
#[derive(Clone, Debug)]
pub struct SearchStats {
    pub candidates_evaluated: u64,
    pub candidates_pruned: u64,
    pub complete_plans_found: u64,
    pub best_score: f32,
    pub stack_depth: usize,
}

/// Compute a fingerprint (hash) of the layer names for cross-tick validation.
pub fn compute_fingerprint(layers: &[Box<dyn PlacementLayer>]) -> u64 {
    let mut hasher = DefaultHasher::new();
    for layer in layers {
        layer.name().hash(&mut hasher);
    }
    hasher.finish()
}
