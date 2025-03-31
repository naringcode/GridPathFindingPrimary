// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:DepthFirstSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Depth-First Search(깊이 우선 탐색)
     * 
     * 가장 깊은 레벨에 있는 노드를 우선적으로 탐색하는 방식이다.
     * FILO(First-In Last-Out) 방식으로 동작하기 때문에 Stack 기반으로 처리해야 한다.
     * 보통은 재귀적 탐색 방식으로 구현하는 경우가 많지만 여기선 탐색 거리를 우선순위 큐에 넣는 방식을 사용할 것이다.
     * 
     * 깊이 단위로 탐색하는 방식으로 가장 마지막에 도달한 노드를 우선적으로 처리한다. 
     */

    // 결과만 도출하는 방식 모음
    // 경로 찾기 과정에서 std::unordered_map 사용
    bool DepthFirstSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
    bool DepthFirstSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 20% 정도 더 좋았음)
    bool DepthFirstSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
    // 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 정도 성능이 더 좋다.
    bool DepthFirstSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext를 이용해 탐색을 진행하는 방식
    bool DepthFirstSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool DepthFirstSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
