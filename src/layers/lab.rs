//! Lab layer: re-exports the lab stamp layer factory from stamp_layer.
//!
//! The lab placement logic is now handled by the generic StampLayer.
//! Use `lab_stamp_layer()` from `stamp_layer` to get a configured StampLayer
//! that places labs near the hub.

pub use crate::layers::stamp_layer::lab_stamp_layer;
