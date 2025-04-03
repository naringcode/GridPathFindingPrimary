// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:JumpPointSearchPlus;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Jump Point Search Plus
     *
     * https://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter14_JPS_Plus_An_Extreme_A_Star_Speed_Optimization_for_Static_Uniform_Cost_Grids.pdf
     * https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter17_Pathfinding_Architecture_Optimizations.pdf
     * https://www.gameaipro.com/GameAIPro3/GameAIPro3_Chapter22_Faster_A_Star_with_Goal_Bounding.pdf
     * https://github.com/SteveRabin/JPSPlusWithGoalBounding/blob/master/Rabin_AISummitGDC2015_JPSPlusWithGoalBounding.pptx
     * 
     * JPS의 단점을 개선한 형태의 알고리즘이다.
     * JPS가 런타임 도중 Jump Points를 생성했다면 JPS+는 사전에 계산한 테이블을 통해 Jump Points를 찾는다.
     * 
     * JPS에서 탐색을 진행하는 것을 보면 Forced Neighbors가 발생하는 위치는 변하지 않는다.
     * 즉, 이러한 위치를 사전에 미리 파악해서 Jump Points의 위치를 미리 계산하는 것(Baking)이 가능하다.
     * 
     * JPS가 런타임 시 Jump Point를 생성했다면 JPS+는 이걸 사전에 미리 계산한 테이블을 사용한다.
     * JPS+는 JPS의 Jump Point 간 탐색 지점을 사전에 계산하여 연결(?)하는 것으로 최적화하는 기법이라고 보면 된다.
     * 
     * --------------------------------------------------
     * 
     * JPS+는 다음 4가지 유형의 Jump Points를 사용한다.
     * 1. Primary Jump Points(사전에 계산된 값)
     * 2. Straight Jump Points(사전에 계산된 값)
     * 3. Diagonal Jump Points(사전에 계산된 값)
     * 4. ★ Target Jump Points(런타임 도중에 계산하여 생성) ★
     * 
     * 
     * 사전에 계산하여 Jump Points를 생성하는 방법은 이러하다.
     * 
     * 1. 막혀 있는 경로로 진입하기 위한 Forced Neighbors의 위치를 미리 파악한다.
     * 
     * 2. 각 Forced Neighbor에 "진입할 수 있는 방향"을 표시하고 이를 Primary Jump Point라고 한다.
     *  - 검색 도중 해당 방향으로 진입해야 Primary Jump Point가 되는 것임.
     *  - 같은 노드를 방문한다고 해도 "진입할 수 있는 방향"으로 들어오지 않으면 Primary Jump Point가 될 수 없음.
     *  - Primary Jump Point는 Cardinal Direction 기준으로 발생하기 때문에 4개의 Flags로 표현할 수 있음.
     * 
     * #. Forced Neighbors를 찾아내는 과정에서 Primary Jump Points를 만드는 것이 효율적이다.
     * 
     * 3. Primary Jump Points에 대한 계산이 끝났으면 그 다음엔 Straight Jump Points를 계산해야 한다.
     *  - Straight Jump Point는 상하좌우 방향을 탐색했을 때 진입하게 되는 Primary Jump Point와의 거리값이 기입됨.
     *  - Primary Jump Point와 만나게 되는 지점이 아닌 "진입할 수 있는" 지점이어야 함.
     *  - (중요) Straight Jump Point 값은 가장 가까운 Primary Jump Point와의 거리를 의미하는 것이 아님.
     * 
     * #. Primary Jump Point와 Straight Jump Point는 별개의 값이다.
     *  - 하지만 노드의 특정 위치는 이러한 별개의 특성을 동시에 표현할 수 있어야 함.
     * 
     * 4. Primary Jump Points와 Straight Jump Points의 계산이 끝났으면 대각선 방향의 Jump Points인 Diagonal Jump Points를 계산해야 한다.
     *  - 이건 대각선 방향의 분해 성분과 관련된 방향(↗ = ↑ + →)으로 마주하게 되는 Straight Jump Point와 관련이 있음.
     *   - Straight Jump Point 자체는 Primary Jump Point로의 진입 방향과 연관성이 있음.
     * 
     * #. 이 부분은 이해하기가 난해할 수 있다.
     *  - 북동쪽(↗)으로 이동할 수 있는 Diagonal Jump Point를 가진 노드가 되기 위해선?
     *   - 이전 대각 위치를 분해했을 때 나오는 북쪽(↑)이나 동쪽(→) 방향으로의 Straight Jump Point에 진입할 수 있어야 함.
     *   - 이건 JPS가 구사하는 대각선 강제이웃 전략과 매우 유사함.
     *   - 이전 대각 방향을 통해 Straight Jump Point로 환승 가능하다면 그 거리가 기입되어야 함.
     *  - 대각선 방향으로 탐색을 진행하면서 Cardinal 방향으로의 환승 지점을 찾는 과정이라고 생각하면 됨.
     * 
     * 5. Jump Points 외 비어있는 부분은 벽과의 거리인 Wall Distances로 채워진다.
     *  - 이 값은 Straight Jump Points와 Diagonal Jump Points를 계산할 때 함께 작성됨.
     *  - 기존 Jump Points와의 값과 구별하기 위해 0과 마이너스 값으로 표기됨.
     *  - 실제로 사용할 때는 음수 기호를 버려야 함.
     * 
     * 
     * 이렇게 맵을 전처리하면 탐색에 필요한 결정 정보를 사전에 생성할 수 있다.
     * 이론만 따지만 Jump Point를 런타임 도중 생성할 필요가 없기 때문에 JPS보다 빠르다.
     * !! 테스트해봤을 때는 OpenSet에 들어가는 노드의 개수에 의해서 성능이 결정되는 경우가 많음. !!
     * 
     * 
     * ★ Target Jump Point는 4가지 유형의 Jump Points 중 유일하게 런타임 시 생성되는 Jump Point이다.
     * - 수직 및 수평 방향 탐색에서 Target Jump Point를 생성하는 것과 대각선 방향 탐색에서 Target Jump Point를 생성하는 것은 다름.
     * - 수직 및 수평 방향 탐색에서 Target Jump Point는 직선 거리 이내에 목표가 있을 경우에만 생성된다.
     * 
     * #. 대각선 방향 탐색에서의 Target Jump Point 생성은 다소 복잡하다.
     * - 작업 위치 기준으로 목표 위치까지의 가로와 세로 거리를 측정해보았을 때,
     *   둘 중 최대 거리가 대각선 방향으로의 벽 또는 다음 점프 지점까지의 거리보다 짧거나 같으면 생성됨.
     * - 이는 런타임 도중 직접 환승 지점(Target Jump Point)을 만들어 목적지까지 직행하는 것을 시도하는 것이라 보면 됨.
     * 
     * 여기서 주의해야 하는 것이 있는데 대각선 방향으로의 Target Jump Point는 목적 지점에 도달할 수 없어도 일단 생성되고 본다는 것이다.
     * 대각선 방향으로 이동 후 수직 및 수평 방향을 통해 목적 지점에 도달할 수 있을 것으로 "예상"된다면 일단 Target JP를 생성하고 본다.
     * 
     * #. 대각선 방향 탐색에서의 Target Jump Point 생성 조건을 충족했으면 다음 순서에 따라 Jump Point를 생성한다.
     *  1. 목표 지점까지의 가로 거리와 세로 거리 중 최솟값을 취함.
     *  2. 그 크기만큼 대각선 방향으로 이동하는 Jump Point를 생성함.
     * 
     * 이때 대각선 탐색 방향에 목표 지점이 있다면?
     *  - 새로운 Target Jump Point의 위치는 목표 노드의 위치와 동일함.
     * 
     * 설명은 복잡하지만 실제 구현된 코드를 보면 쉽게 이해할 수 있다.
     * 맵을 전처리 혹은 Baking하는 과정과 이를 통해 탐색하는 과정 자체는 별개의 것임을 구분해서 봐야 한다.
     */

    // 결과만 도출하는 방식 모음
    // 경로 찾기 과정에서 std::unordered_map 사용
    bool JumpPointSearchPlus1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
    bool JumpPointSearchPlus2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 20% 정도 더 좋았음)
    bool JumpPointSearchPlus3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
    // 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 정도 성능이 더 좋다.
    bool JumpPointSearchPlus4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);
    
    // PathFindingContext를 이용해 탐색을 진행하는 방식
    bool JumpPointSearchPlusAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool JumpPointSearchPlusComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
