// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:DijkstraSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Dijkstra
     * 
     * f(n) = g(n)
     * 이동 거리를 합산해서 거리가 가장 짧을 것으로 예상되는 노드를 우선적으로 탐색하는 방식이다.
     * 비용은 비싸도 이론만 따지면 모든 정점 간의 최단 거리를 구할 수 있다.
     */
    
    // 결과만 도출하는 방식 모음
    // 경로 찾기 과정에서 std::unordered_map 사용
    bool DijkstraSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
    bool DijkstraSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 20% 정도 더 좋았음)
    bool DijkstraSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
    // 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 정도 성능이 더 좋다.
    bool DijkstraSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext를 이용해 탐색을 진행하는 방식
    bool DijkstraSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool DijkstraSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
