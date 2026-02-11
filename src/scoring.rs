//! Scoring re-exports.
//!
//! Scoring is now modeled as individual placement layers (see `layers::anchor_score`,
//! `layers::hub_quality_score`, `layers::extension_score`, etc.). This module
//! re-exports the PlanScore type for backward compatibility.

pub use crate::layer::ScoreEntry;
pub use crate::plan::PlanScore;
