#include <stddef.h>
#include <stdint.h>

#include "decision.h"
#include "trajectory.h"

#if defined(_WIN32)
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT __attribute__((visibility("default")))
#endif

SIM_EXPORT uint32_t Simulation_SizeOfDecisionPoint(void)
{
    return (uint32_t)sizeof(DecisionPoint);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionMove(void)
{
    return (uint32_t)sizeof(DecisionMove);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionPlan(void)
{
    return (uint32_t)sizeof(DecisionPlan);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionCardEdgeEvent(void)
{
    return (uint32_t)sizeof(DecisionCardEdgeEvent);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionCardPrimitive(void)
{
    return (uint32_t)sizeof(DecisionCardPrimitive);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionCardPieceFeatures(void)
{
    return (uint32_t)sizeof(DecisionCardPieceFeatures);
}

SIM_EXPORT uint32_t Simulation_SizeOfDecisionCardFrame(void)
{
    return (uint32_t)sizeof(DecisionCardFrame);
}

SIM_EXPORT uint32_t Simulation_SizeOfTrajectoryPose(void)
{
    return (uint32_t)sizeof(TrajectoryPose);
}

SIM_EXPORT uint32_t Simulation_SizeOfTrajectoryPlan(void)
{
    return (uint32_t)sizeof(TrajectoryPlan);
}

SIM_EXPORT uint32_t Simulation_OffsetOfDecisionPlanMoves(void)
{
    return (uint32_t)offsetof(DecisionPlan, moves);
}

SIM_EXPORT uint32_t Simulation_OffsetOfDecisionPlanSearchNodes(void)
{
    return (uint32_t)offsetof(DecisionPlan, search_nodes);
}

SIM_EXPORT float Simulation_DecisionAssemblyClearanceMm(void)
{
    return DECISION_ASSEMBLY_CLEARANCE_MM;
}
