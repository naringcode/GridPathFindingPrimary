// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:BreadthFirstSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Breadth-First Search(너비 우선 탐색)
     * 
     * 너비를 확장하며 탐색하는 방식을 말하며 너비 단위의 FIFO(First-In First-Out) 기반으로 동작한다.
     */
    
    // 결과만 도출하는 방식 모음
    // 경로 찾기 과정에서 std::unordered_map 사용
    bool BreadthFirstSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
    bool BreadthFirstSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 20% 정도 더 좋았음)
    bool BreadthFirstSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
    // 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 정도 성능이 더 좋다.
    bool BreadthFirstSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext를 이용해 탐색을 진행하는 방식
    bool BreadthFirstSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool BreadthFirstSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
