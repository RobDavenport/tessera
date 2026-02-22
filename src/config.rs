use crate::bsp::BspConfig;
use crate::cellular::CellularConfig;
use crate::drunkard::DrunkardConfig;
use crate::poisson::PoissonConfig;

#[derive(Clone, Debug, Default)]
pub struct GenerationConfig {
    pub bsp: BspConfig,
    pub cellular: CellularConfig,
    pub drunkard: DrunkardConfig,
    pub poisson: PoissonConfig,
}
