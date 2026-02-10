pub mod anchor;
pub mod anchor_score;
pub mod lab;
pub mod stamp_layer;
pub mod spawn;
pub mod tower;
pub mod utility;
pub mod source_infra;
pub mod controller_infra;
pub mod mineral_infra;
pub mod extension;
pub mod road_network;
pub mod road_prune;
pub mod reachability;
pub mod defense;
pub mod rcl_assignment;
pub mod hub_quality_score;
pub mod upgrade_area_score;
pub mod extension_score;
pub mod tower_coverage_score;
pub mod upkeep_score;

pub use anchor::AnchorLayer;
pub use anchor_score::AnchorScoreLayer;
pub use stamp_layer::{StampLayer, GreedyStampLayer, hub_stamp_layer, lab_stamp_layer};
pub use spawn::SpawnLayer;
pub use tower::TowerLayer;
pub use utility::UtilityLayer;
pub use source_infra::SourceInfraLayer;
pub use controller_infra::ControllerInfraLayer;
pub use mineral_infra::MineralInfraLayer;
pub use extension::ExtensionLayer;
pub use road_network::RoadNetworkLayer;
pub use road_prune::RoadPruneLayer;
pub use reachability::ReachabilityLayer;
pub use defense::DefenseLayer;
pub use rcl_assignment::RclAssignmentLayer;
pub use hub_quality_score::HubQualityScoreLayer;
pub use upgrade_area_score::UpgradeAreaScoreLayer;
pub use extension_score::ExtensionScoreLayer;
pub use tower_coverage_score::TowerCoverageScoreLayer;
pub use upkeep_score::UpkeepScoreLayer;

use crate::layer::PlacementLayer;

/// Build the default layer stack (21 layers).
///
/// 1. AnchorLayer -- position selection only, sets "hub" landmark
/// 2. hub_stamp_layer -- StampLayer placing hub stamp at "hub", radius 0
/// 3. AnchorScoreLayer -- source/ctrl/exit/mineral distance
/// 4. HubQualityScoreLayer -- key structures adjacent to hub
/// 5. lab_stamp_layer -- StampLayer placing lab stamp near "hub", radius 8
/// 6. SpawnLayer -- additional spawns
/// 7. TowerLayer -- greedy tower placement
/// 8. UtilityLayer -- observer
/// 9. SourceInfraLayer -- containers + links (fails on missing)
/// 10. ControllerInfraLayer -- upgrade area + container + link (fails on missing)
/// 11. UpgradeAreaScoreLayer
/// 12. MineralInfraLayer -- extractor + container
/// 13. ExtensionLayer -- target 60, min 60 (fails if < min)
/// 14. ExtensionScoreLayer
/// 15. RoadNetworkLayer
/// 16. RoadPruneLayer -- removes dead-end and unreachable roads
/// 17. ReachabilityLayer -- BFS validates all structures reachable from hub
/// 18. DefenseLayer
/// 19. RclAssignmentLayer
/// 20. TowerCoverageScoreLayer
/// 21. UpkeepScoreLayer
pub fn default_layers() -> Vec<Box<dyn PlacementLayer>> {
    vec![
        Box::new(AnchorLayer::default()),
        Box::new(hub_stamp_layer()),
        Box::new(AnchorScoreLayer),
        Box::new(HubQualityScoreLayer),
        Box::new(lab_stamp_layer()),
        Box::new(SpawnLayer),
        Box::new(TowerLayer),
        Box::new(UtilityLayer),
        Box::new(SourceInfraLayer),
        Box::new(ControllerInfraLayer),
        Box::new(UpgradeAreaScoreLayer),
        Box::new(MineralInfraLayer),
        Box::new(ExtensionLayer::default()),
        Box::new(ExtensionScoreLayer),
        Box::new(RoadNetworkLayer),
        Box::new(RoadPruneLayer),
        Box::new(ReachabilityLayer),
        Box::new(DefenseLayer),
        Box::new(RclAssignmentLayer),
        Box::new(TowerCoverageScoreLayer),
        Box::new(UpkeepScoreLayer),
    ]
}
