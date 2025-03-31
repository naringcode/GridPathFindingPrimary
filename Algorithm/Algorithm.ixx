// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm;

export import :Core;
export import :Heuristic;

export import :DepthFirstSearch;
export import :BreadthFirstSearch;
export import :BestFirstSearch;
export import :DijkstraSearch;
export import :AStarSearch;
export import :JumpPointSearch;
export import :JumpPointSearchPlus;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class PathFindingContext;

    bool DoPathFindingAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool DoPathFindingComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
module: private;
