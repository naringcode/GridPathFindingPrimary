// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:JumpPointSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Jump Point Search
     * 
     * https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search/
     * https://harablog.wordpress.com/2011/09/07/jump-point-search/
     * https://www.gamedev.net/articles/programming/artificial-intelligence/jump-point-search-fast-a-pathfinding-for-uniform-cost-grids-r4220/
     * 
     * Jump Point를 탐색하는 방식으로 동작하는 길찾기 알고리즘이며
     * 다음 두 가지 요소는 JPS의 핵심이 되는 부분이다.
     * 
     * 1. 가지치기(Pruning)
     *  - A*처럼 모든 방향을 탐색하는 것이 아닌 불필요한 방향은 제외하고 특정 방향으로 탐색하는 방법을 말함.
     *  - 이를 통해 OpenSet에 노드를 넣는 행위를 최소화할 수 있음.
     *  - 가지치기를 통한 탐색은 강제 이웃을 만나기 전가지 계속됨.
     *
     * 2. 강제 이웃(Forced Neighbor)
     *  - 막혀 있는 경로에 진입하기 위해 방문 및 우회해야 하는 이웃을 말함.
     *  - 보통 코너 지점에서 발생하며 강제 이웃을 탐지하면 기존 방향으로의 탐색을 멈추고 탐색 권한을 강제 이웃에게 위임함.
     *  - 강제 이웃의 위치에 있는 노드는 Jump Point로 사용되어야 하기에 OpenSet에 들어감.
     * 
     * 3. Jump Point
     *  - 건너뛰어서 탐색을 재개하는 지점을 말함.
     *  - 이전 방향과 막힌 길을 우회하기 위해 생기는 가지 방향으로의 탐색을 이어서 진행함.
     * 
     * (주의) 강제 이웃은 Jump Point가 아니며, 이는 Jump Point가 될 수 있는 가능성을 가진 후보를 말한다(모든 Forced Neighbor가 JP인 것은 아님).
     * 
     * JPS는 한 방향으로의 탐색 과정에서 발생하는 불필요한 OpenSet으로의 삽입 과정을 생략하여 탐색을 최적화한 알고리즘이다.
     * JPS 자체는 8방향 탐색을 기본 골자로 하고 있기 때문에 대각성 이동을 허용하는 것이 좋다.
     * 또한 가지치기 전략을 간소화하기 위해 코너 이동 여부를 확실하게 정하고 가도록 한다.
     * 
     * 만약 대각선 이동이나 코너 이동을 허용하지 않는다면 별도의 탐색 함수를 따로 구성해야 할 수도 있다.
     * 대각선 + 코너 허용, 대각선 + 코너 미허용, 대각선 미허용 전부 Jump Point 생성 전략이 상이하다.
     */
    
    // 결과만 도출하는 방식 모음
    // 경로 찾기 과정에서 std::unordered_map 사용
    bool JumpPointSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
    bool JumpPointSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 20% 정도 더 좋았음)
    bool JumpPointSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
    // 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 정도 성능이 더 좋다.
    bool JumpPointSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    
    // PathFindingContext를 이용해 탐색을 진행하는 방식
    bool JumpPointSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool JumpPointSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
