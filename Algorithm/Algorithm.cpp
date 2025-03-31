// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Algorithm;

import PathFinding.Context;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

bool DoPathFindingAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    switch (pathFindingContext->GetAlgorithmType())
    {
        case AlgorithmType::DepthFirst:
            return DepthFirstSearchAdvance(pathFindingContext);

        case AlgorithmType::BreadthFirst:
            return BreadthFirstSearchAdvance(pathFindingContext);

        case AlgorithmType::BestFirst:
            return BestFirstSearchAdvance(pathFindingContext);

        case AlgorithmType::Dijkstra:
            return DijkstraSearchAdvance(pathFindingContext);

        case AlgorithmType::AStar:
            return AStarSearchAdvance(pathFindingContext);

        case AlgorithmType::JumpPoint:
            return JumpPointSearchAdvance(pathFindingContext);

        case AlgorithmType::JumpPointPlus:
            return JumpPointSearchPlusAdvance(pathFindingContext);
    }

    return false;
}

bool DoPathFindingComplete(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    switch (pathFindingContext->GetAlgorithmType())
    {
        case AlgorithmType::DepthFirst:
            return DepthFirstSearchComplete(pathFindingContext);

        case AlgorithmType::BreadthFirst:
            return BreadthFirstSearchComplete(pathFindingContext);

        case AlgorithmType::BestFirst:
            return BestFirstSearchComplete(pathFindingContext);

        case AlgorithmType::Dijkstra:
            return DijkstraSearchComplete(pathFindingContext);

        case AlgorithmType::AStar:
            return AStarSearchComplete(pathFindingContext);

        case AlgorithmType::JumpPoint:
            return JumpPointSearchComplete(pathFindingContext);

        case AlgorithmType::JumpPointPlus:
            return JumpPointSearchPlusComplete(pathFindingContext);
    }

    return false;
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
