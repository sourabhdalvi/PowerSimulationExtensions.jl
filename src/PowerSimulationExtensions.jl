module PowerSimulationExtensions

#################################################################################
# Exports
# OperationProblem
export MILPDualProblem
export StandardUnitCommitmentCC
export MILPDualProblemCC

# Service Formulations
export RampReserve
export QuadraticCostRampReserve
export InertiaReserve

# Device Formulations
export ThermalInertiaBasicUnitCommitment
export ThermalInertiaStandardUnitCommitment
export HydroInertiaCommitmentRunOfRiver
export BookKeepingwInertia
export RenewableFullDispatchInertia

# Branch Formulations
export StaticBranchFlowCost

#FeedForwards
export InertiaFF
export EnergyTargetFF
export ReserveSemiContinuousFF

#################################################################################
# Imports
import PowerSystems
import PowerSimulations
import JuMP
import DataFrames
import ParameterJuMP
import InfrastructureSystems
import Dates
import MathOptInterface
# import Gurobi

const IS = InfrastructureSystems
const MOI = MathOptInterface
const PSY = PowerSystems
const PSI = PowerSimulations
const PM = PowerSimulations.PM
const PJ = ParameterJuMP

#################################################################################
# Includes
include("./core/definitions.jl")

include("device_models/devices/common/constraints_structs.jl")
include("device_models/devices/common/rateofchange_constraints.jl")
include("device_models/devices/common/cost_fuctions.jl")
include("device_models/devices/common/commitment_constraint.jl")
include("device_models/devices/common/inertia_constraints.jl")

include("./core/operations_problem.jl")
include("./core/optimization_container.jl")
include("./service_models/reserves.jl")
include("./core/feedforward.jl")

include("device_models/devices/thermal_generation.jl")
include("device_models/devices/renewable_generation.jl")
include("device_models/devices/storage.jl")
include("device_models/devices/hydro_generation.jl")
include("device_models/devices/AC_branches.jl")

include("./service_models/services_constructor.jl")
include("./device_models/device_constructors/thermalgeneration_constructor.jl")
include("./device_models/device_constructors/renewablegeneration_constructor.jl")
include("./device_models/device_constructors/storage_constructor.jl")
include("./device_models/device_constructors/hydrogeneration_constructor.jl")
include("./device_models/device_constructors/branch_constructor.jl")

end # module
