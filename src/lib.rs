pub mod constants;
pub mod planner;
pub mod scoring;
pub mod layout;
pub mod location;

#[cfg(feature = "shim")]
pub mod shim;
#[cfg(feature = "shim")]
pub use shim::*;

pub mod visual;
pub use visual::*;

#[cfg(not(feature = "shim"))]
pub use screeps::*;