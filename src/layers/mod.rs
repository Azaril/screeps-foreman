pub mod anchor;
pub mod anchor_score;
pub mod controller_infra;
pub mod defense;
pub mod exit_setback;
pub mod extension;
pub mod extension_score;
pub mod hub_quality_score;
pub mod lab;
pub mod mineral_infra;
pub mod rcl_assignment;
pub mod reachability;
pub mod road_network;
pub mod road_prune;
pub mod source_infra;
pub mod spawn;
pub mod stamp_layer;
pub mod tower;
pub mod tower_coverage_score;
pub mod upgrade_area_score;
pub mod upkeep_score;
pub mod utility;

pub use anchor::AnchorLayer;
pub use anchor_score::AnchorScoreLayer;
pub use controller_infra::ControllerInfraLayer;
pub use defense::DefenseLayer;
pub use exit_setback::ExitSetbackLayer;
pub use extension::ExtensionLayer;
pub use extension_score::ExtensionScoreLayer;
pub use hub_quality_score::HubQualityScoreLayer;
pub use mineral_infra::MineralInfraLayer;
pub use rcl_assignment::RclAssignmentLayer;
pub use reachability::ReachabilityLayer;
pub use road_network::RoadNetworkLayer;
pub use road_prune::RoadPruneLayer;
pub use source_infra::SourceInfraLayer;
pub use spawn::SpawnLayer;
pub use stamp_layer::{
    hub_stamp_layer, lab_stamp_layer, GreedyStampLayer, ScoredStampLayer, StampLayer,
};
pub use tower::TowerLayer;
pub use tower_coverage_score::TowerCoverageScoreLayer;
pub use upgrade_area_score::UpgradeAreaScoreLayer;
pub use upkeep_score::UpkeepScoreLayer;
pub use utility::UtilityLayer;

use crate::layer::PlacementLayer;

/// Build the default layer stack (23 layers).
///
///  1. ExitSetbackLayer -- excludes tiles within 1 of exit tiles from placement
///  2. AnchorLayer -- position selection only, sets "hub" landmark
///  3. hub_stamp_layer -- StampLayer placing hub stamp at "hub", radius 0
///  4. AnchorScoreLayer -- source/ctrl/exit/mineral distance
///  5. HubQualityScoreLayer -- key structures adjacent to hub
///  6. lab_stamp_layer -- StampLayer placing lab stamp near "hub", radius 8
///  7. SpawnLayer -- additional spawns
///  8. TowerLayer -- greedy tower placement
///  9. UtilityLayer -- observer
/// 10. SourceInfraLayer -- containers + links (fails on missing)
/// 11. ControllerInfraLayer -- upgrade area + container + link (fails on missing)
/// 12. UpgradeAreaScoreLayer
/// 13. MineralInfraLayer -- extractor + container
/// 14. RoadNetworkLayer::infrastructure() -- early roads to sources/controller
/// 15. ExtensionLayer -- target 60, min 60 (fails if < min)
/// 16. ExtensionScoreLayer
/// 17. DefenseLayer -- ramparts/walls (before all_buildings so roads route through ramparts)
/// 18. RoadNetworkLayer::all_buildings() -- roads to all remaining interactable buildings
/// 19. RoadPruneLayer -- removes dead-end, unreachable, and redundant roads
/// 20. ReachabilityLayer -- BFS validates all structures reachable from hub
/// 21. RclAssignmentLayer
/// 22. TowerCoverageScoreLayer
/// 23. UpkeepScoreLayer
pub fn default_layers() -> Vec<Box<dyn PlacementLayer>> {
    vec![
        Box::new(ExitSetbackLayer::new(1)),
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
        Box::new(RoadNetworkLayer::infrastructure()),
        Box::new(ExtensionLayer::default()),
        Box::new(ExtensionScoreLayer),
        Box::new(DefenseLayer),
        Box::new(RoadNetworkLayer::all_buildings()),
        Box::new(RoadPruneLayer),
        Box::new(ReachabilityLayer),
        Box::new(RclAssignmentLayer),
        Box::new(TowerCoverageScoreLayer),
        Box::new(UpkeepScoreLayer),
    ]
}
