// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Algorithm:JumpPointSearchPlus;

import PointerPriorityQueue;

import PathFinding.Map;
import PathFinding.Context;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

/**
 * https://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter14_JPS_Plus_An_Extreme_A_Star_Speed_Optimization_for_Static_Uniform_Cost_Grids.pdf
 * 
 * 위 링크에 나온 것을 토대로 하면 Flags를 통한 분기 방식이 아닌 DirLookUpTable을 기반으로 동작한다.
 * 여기서는 AdjacentDirection을 이동할 방향에 대한 값과 인덱스로 사용한다.
 * 
 * 플래그 방식도 불가능한 것은 아니지만 이를 테이블로 구성하려면 배열을 256(AllFlags + 1)만큼 할당해야 한다.
 * 그리고 모든 탐색 방향에 대한 값을 하나하나 지정해줘야 한다.
 * 
 * 위 링크에서는 방향 테이블을 기반으로 탐색하지만 잘 구현하면 테이블을 사용하지 않고
 * 직접 구현한 JPS와 유사하게 플래그로도 충분히 JPS+의 탐색 경로를 구현할 수 있다.
 * 
 * 하지만 JPS+의 핵심은 강제 이웃을 런타임에 계산하는 것이 아닌 사전에 계산하고 이를 사용하는 것이기 때문에 
 */
// static std::vector<AdjacentDirection> g_DirLookUpTable[] =
// {
//     // NorthWest
//     { AdjacentDirection::West, AdjacentDirection::NorthWest, AdjacentDirection::North },
// 
//     // North
//     { AdjacentDirection::West, AdjacentDirection::NorthWest, AdjacentDirection::North, AdjacentDirection::NorthEast, AdjacentDirection::East },
// 
//     // NorthEast
//     { AdjacentDirection::North, AdjacentDirection::NorthEast, AdjacentDirection::East },
// 
//     // West
//     { AdjacentDirection::South, AdjacentDirection::SouthWest, AdjacentDirection::West, AdjacentDirection::NorthWest, AdjacentDirection::North },
// 
//     // None : 이 값은 모든 방향을 지정하는데 사용한다.
//     { AdjacentDirection::North, AdjacentDirection::NorthEast, AdjacentDirection::East, AdjacentDirection::SouthEast, AdjacentDirection::South, AdjacentDirection::SouthWest, AdjacentDirection::West, AdjacentDirection::NorthWest },
// 
//     // East
//     { AdjacentDirection::North, AdjacentDirection::NorthEast, AdjacentDirection::East, AdjacentDirection::SouthEast, AdjacentDirection::South },
// 
//     // SouthWest
//     { AdjacentDirection::South, AdjacentDirection::SouthWest, AdjacentDirection::West },
// 
//     // South
//     { AdjacentDirection::East, AdjacentDirection::SouthEast, AdjacentDirection::South, AdjacentDirection::SouthWest, AdjacentDirection::West },
// 
//     // SouthEast
//     { AdjacentDirection::East, AdjacentDirection::SouthEast, AdjacentDirection::South }
// };

bool JumpPointSearchPlus1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+를 위한 최적화가 되어 있어야 한다.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;
    
    // 위치에 해당하는 노드를 가져오기 위한 자료형
    std::unordered_map<i32, PathFindingNode> posToPathFindingNodeMap;

    // posToPathFindingNodeMap에서 가져온 노드를 처리할 우선순위 큐를 만들고 최초 OpenSet 삽입 진행
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;
    
    tls_PathFindingNodePQ.Clear();
    
    // 최초 OpenSet에 삽입
    PathFindingNode* startNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None은 모든 방향으로 매핑된다.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet = true;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
        
        // 작업 노드는 OpenSet에서 빠져나오며 ClosedSet에 들어간다.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
    
        // 목표 도달
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // PathFindingNode가 아닌 Jump Points나 Wall과의 거리가 기입된 노드이다.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // 목표 지점까지 수직 + 수평 거리
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // 인접 노드를 조회하며 OpenSet 갱신
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // 상하좌우
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // 정확한 방향에 목표 지점 존재
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // 직선 거리 이내에 목표 존재
            {
                // 원문 : Goal is closer than wall distance or closer than or equal to jump point distance.
                // 점프 해야 하는 위치가 목표 지점이다.
                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // 대각선
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // 사분면 안에 목표 지점 존재
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // 가로 거리 이내에 목표가 있거나
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // 세로 거리 이내에 목표가 존재
            {
                // 원문 : Goal is closer or equal in either row or column than wall or jump point distance.
                // 가로 혹은 세로 거리 이내에 목표가 있다면 환승 지점(Target Jump Point)을 구성한다.
                i32        minDiff = std::min(destDx, destDy); // 한 번 꺾어서 도달할 것이기 때문에 가로 거리와 세로 거리 중 값이 작은 것은 선택함.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir 방향에 minDiff를 곱하여 움직일 거리를 계산함.
                
                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // 해당 방향에 Jump Point가 있으면 JP가 있는 위치로 이동한다(벽은 음수로 평가되기에 안 들어 옮).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir 방향에 JP와의 거리를 곱하여 움직일 거리를 계산함.

                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // 기본 방위 방향은 단위가 1이지만 대각선은 따로 처리해야 한다.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // 다음에 이동할 Jump Point를 찾은 상태
            if (nextNode != nullptr)
            {
                // 이미 사용된 노드
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // 새로운 노드
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet에 추가
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
                    nextNode->parent = processingNode;

                    // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // 갱신
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// 경로 찾기 과정에서 thread_local std::unordered_map 사용(1번 방식보다 성능이 대략 15% 정도 더 좋았음)
bool JumpPointSearchPlus2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+를 위한 최적화가 되어 있어야 한다.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    // 위치에 해당하는 노드를 가져오기 위한 자료형
    static thread_local std::unordered_map<i32, PathFindingNode> tls_PosToPathFindingNodeMap;

    tls_PosToPathFindingNodeMap.clear();

    // tls_PosToPathFindingNodeMap에서 가져온 노드를 처리할 우선순위 큐를 만들고 최초 OpenSet 삽입 진행
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;

    tls_PathFindingNodePQ.Clear();

    // 최초 OpenSet에 삽입
    PathFindingNode* startNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->parent = nullptr;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None은 모든 방향으로 매핑된다.
        startNode->indegreeDirection = AdjacentDirection::None;
        
        startNode->isInOpenSet = true;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
        
        // 작업 노드는 OpenSet에서 빠져나오며 ClosedSet에 들어간다.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
    
        // 목표 도달
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // PathFindingNode가 아닌 Jump Points나 Wall과의 거리가 기입된 노드이다.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // 목표 지점까지 수직 + 수평 거리
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // 인접 노드를 조회하며 OpenSet 갱신
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // 상하좌우
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // 정확한 방향에 목표 지점 존재
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // 직선 거리 이내에 목표 존재
            {
                // 원문 : Goal is closer than wall distance or closer than or equal to jump point distance.
                // 점프 해야 하는 위치가 목표 지점이다.
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // 대각선
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // 사분면 안에 목표 지점 존재
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // 가로 거리 이내에 목표가 있거나
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // 세로 거리 이내에 목표가 존재
            {
                // 원문 : Goal is closer or equal in either row or column than wall or jump point distance.
                // 가로 혹은 세로 거리 이내에 목표가 있다면 환승 지점(Target Jump Point)을 구성한다.
                i32        minDiff = std::min(destDx, destDy); // 한 번 꺾어서 도달할 것이기 때문에 가로 거리와 세로 거리 중 값이 작은 것은 선택함.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir 방향에 minDiff를 곱하여 움직일 거리를 계산함.
                
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // 해당 방향에 Jump Point가 있으면 JP가 있는 위치로 이동한다(벽은 음수로 평가되기에 안 들어 옮).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir 방향에 JP와의 거리를 곱하여 움직일 거리를 계산함.

                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // 기본 방위 방향은 단위가 1이지만 대각선은 따로 처리해야 한다.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // 다음에 이동할 Jump Point를 찾은 상태
            if (nextNode != nullptr)
            {
                // 이미 사용된 노드
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // 새로운 노드
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet에 추가
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
                    nextNode->parent = processingNode;

                    // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // 갱신
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// 경로 찾기 과정에서 thread_local std::unordered_map에 탐색 아이디 적용(2번 방식보다 성능이 대략 성능이 대략 20% 정도 더 좋았음)
bool JumpPointSearchPlus3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+를 위한 최적화가 되어 있어야 한다.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;
    
    // 길찾기 아이디 발급기
    static thread_local ui32 pathFindingIdGenerator = 0;
    
    const ui32 kPathFindingId = pathFindingIdGenerator++;

    // 위치에 해당하는 노드를 가져오기 위한 자료형
    static thread_local std::unordered_map<i32, PathFindingNode> tls_PosToPathFindingNodeMap;

    // 길찾기 아이디 방식으로 노드를 식별하면 clear()를 호출하지 않아도 할당된 노드를 재사용할 수 있다.
    // tls_PosToPathFindingNodeMap.clear();

    // tls_PosToPathFindingNodeMap에서 가져온 노드를 처리할 우선순위 큐를 만들고 최초 OpenSet 삽입 진행
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;
    
    tls_PathFindingNodePQ.Clear();

    // 최초 OpenSet에 삽입(완전 초기화 진행)
    PathFindingNode* startNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->parent = nullptr;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None은 모든 방향으로 매핑된다.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
        
        // 작업 노드는 OpenSet에서 빠져나오며 ClosedSet에 들어간다.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
    
        // 목표 도달
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // PathFindingNode가 아닌 Jump Points나 Wall과의 거리가 기입된 노드이다.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // 목표 지점까지 수직 + 수평 거리
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // 인접 노드를 조회하며 OpenSet 갱신
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // 상하좌우
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // 정확한 방향에 목표 지점 존재
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // 직선 거리 이내에 목표 존재
            {
                // 원문 : Goal is closer than wall distance or closer than or equal to jump point distance.
                // 점프 해야 하는 위치가 목표 지점이다.
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // 대각선
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // 사분면 안에 목표 지점 존재
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // 가로 거리 이내에 목표가 있거나
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // 세로 거리 이내에 목표가 존재
            {
                // 원문 : Goal is closer or equal in either row or column than wall or jump point distance.
                // 가로 혹은 세로 거리 이내에 목표가 있다면 환승 지점(Target Jump Point)을 구성한다.
                i32        minDiff = std::min(destDx, destDy); // 한 번 꺾어서 도달할 것이기 때문에 가로 거리와 세로 거리 중 값이 작은 것은 선택함.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir 방향에 minDiff를 곱하여 움직일 거리를 계산함.
                
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // 해당 방향에 Jump Point가 있으면 JP가 있는 위치로 이동한다(벽은 음수로 평가되기에 안 들어 옮).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir 방향에 JP와의 거리를 곱하여 움직일 거리를 계산함.

                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // 기본 방위 방향은 단위가 1이지만 대각선은 따로 처리해야 한다.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // 다음에 이동할 Jump Point를 찾은 상태
            if (nextNode != nullptr)
            {
                // 길찾기 아이디가 일치하지 않으면 새로운 탐색으로 간주한다.
                if (nextNode->pathFindingId != kPathFindingId)
                {
                    nextNode->isInOpenSet   = false;
                    nextNode->isInClosedSet = false;

                    nextNode->pathFindingId = kPathFindingId;
                }

                // 이미 사용된 노드
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // 새로운 노드
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet에 추가
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
                    nextNode->parent = processingNode;

                    // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // 갱신
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// 경로 찾기 과정에서 thread_local std::vector에 탐색 아이디 사용(이건 매번 노드를 맵의 크기만큼 할당할 수 없기 때문에 thread_local이 필수임)
// 3번 방식보다 성능이 대략 20% 정도 더 좋으며 1번 방식과 비교하면 거의 40% 성능이 더 좋다.
bool JumpPointSearchPlus4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+를 위한 최적화가 되어 있어야 한다.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    // 길찾기 아이디 발급기
    static thread_local ui32 pathFindingIdGenerator = 0;

    const ui32 kPathFindingId = pathFindingIdGenerator++;

    // 위치에 해당하는 노드를 가져오기 위한 자료형
    static thread_local std::vector<PathFindingNode> tls_PathFindingNodes;

    // 길찾기 아이디 방식으로 노드를 식별하면 clear()를 호출하지 않아도 할당된 노드를 재사용할 수 있다.
    // tls_PathFindingNodes.clear();

    // std::vector()의 resize()는 새로운 크기가 기존 크기보다 클 경우에만 영향을 미친다.
    tls_PathFindingNodes.resize(staticMap->GetSize());

    // tls_PathFindingNodes에서 가져온 노드를 처리할 우선순위 큐를 만들고 최초 OpenSet 삽입 진행
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;

    tls_PathFindingNodePQ.Clear();

    // 최초 OpenSet에 삽입(완전 초기화 진행)
    PathFindingNode* startNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->parent = nullptr;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None은 모든 방향으로 매핑된다.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
        
        // 작업 노드는 OpenSet에서 빠져나오며 ClosedSet에 들어간다.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
    
        // 목표 도달
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // PathFindingNode가 아닌 Jump Points나 Wall과의 거리가 기입된 노드이다.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // 목표 지점까지 수직 + 수평 거리
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // 인접 노드를 조회하며 OpenSet 갱신
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // 상하좌우
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // 정확한 방향에 목표 지점 존재
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // 직선 거리 이내에 목표 존재
            {
                // 원문 : Goal is closer than wall distance or closer than or equal to jump point distance.
                // 점프 해야 하는 위치가 목표 지점이다.
                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // 대각선
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // 사분면 안에 목표 지점 존재
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // 가로 거리 이내에 목표가 있거나
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // 세로 거리 이내에 목표가 존재
            {
                // 원문 : Goal is closer or equal in either row or column than wall or jump point distance.
                // 가로 혹은 세로 거리 이내에 목표가 있다면 환승 지점(Target Jump Point)을 구성한다.
                i32        minDiff = std::min(destDx, destDy); // 한 번 꺾어서 도달할 것이기 때문에 가로 거리와 세로 거리 중 값이 작은 것은 선택함.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir 방향에 minDiff를 곱하여 움직일 거리를 계산함.
                
                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // 해당 방향에 Jump Point가 있으면 JP가 있는 위치로 이동한다(벽은 음수로 평가되기에 안 들어 옮).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir 방향에 JP와의 거리를 곱하여 움직일 거리를 계산함.

                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // 기본 방위 방향은 단위가 1이지만 대각선은 따로 처리해야 한다.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // 다음에 이동할 Jump Point를 찾은 상태
            if (nextNode != nullptr)
            {
                // 길찾기 아이디가 일치하지 않으면 새로운 탐색으로 간주한다.
                if (nextNode->pathFindingId != kPathFindingId)
                {
                    nextNode->isInOpenSet   = false;
                    nextNode->isInClosedSet = false;

                    nextNode->pathFindingId = kPathFindingId;
                }

                // 이미 사용된 노드
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // 새로운 노드
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet에 추가
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
                    nextNode->parent = processingNode;

                    // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // 갱신
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

bool JumpPointSearchPlusAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    if (pathFindingContext->GetAlgorithmType() != AlgorithmType::JumpPointPlus)
        return false;

    PathFindingContext::Phase phase = pathFindingContext->GetPhase();
    PathFindingRecord pathFindingRecord;

    std::shared_ptr<StaticMap> staticMap = pathFindingContext->GetStaticMap();
    Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

    switch (phase)
    {
        case PathFindingContext::Phase::Ready:
        {
            // JPS+는 대각선을 허용할 때만 동작한다.
            if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
            {
                pathFindingContext->FinishPathFinding(false);

                return false;
            }

            // JPS+를 위한 최적화가 되어 있어야 한다.
            if (false == staticMap->IsOptimizedForJpsPlus())
            {
                pathFindingContext->FinishPathFinding(false);

                return false;
            }

            // 최초 OpenSet에 삽입
            Vec2D<i32> startPos = pathFindingContext->GetStartPos();
            Vec2D<i32> destPos  = pathFindingContext->GetDestinationPos();
            
            PathFindingNode* startNode = pathFindingContext->GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
            {
                startNode->g = 0.0f;
                startNode->h = CalculateHeuristicCost(pathFindingContext->GetHeuristicType(), (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), pathFindingContext->GetHeuristicWeight());
                startNode->f = startNode->h;

                // None은 모든 방향으로 매핑된다.
                startNode->indegreeDirection = AdjacentDirection::None;

                startNode->isInOpenSet = true;
            }
            
            // OpenList(OpenSet)에 추가하는 것은 처리한 것이라 보지 않는다.
            pathFindingRecord.SetProcessingNode(-1, -1, -1, -1, 0.0f, 0.0f, 0.0f);

            pathFindingRecord.AddVisitedNode(startPos.x, startPos.y, startPos.x, startPos.y, startNode->f, 0.0f, startNode->h);
            pathFindingRecord.AddOpenNewNode(startPos.x, startPos.y, startPos.x, startPos.y, startNode->f, 0.0f, startNode->h);

            pathFindingContext->RecordPathFinding(&pathFindingRecord);

            pathFindingContext->StartPathFinding(*startNode);

            return true;
        }

        case PathFindingContext::Phase::PathFinding:
        {
            // 목표에 도달할 때까지 길찾기 로직 수행
            std::shared_ptr<StaticMap> staticMap = pathFindingContext->GetStaticMap();
            Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

            PathFindingNode* processingNode;
            if (pathFindingContext->DequeuePathFindingNode(&processingNode) == false)
            {
                // 경로 찾기 실패했으며 길찾기 작업을 종료한다.
                pathFindingContext->FinishPathFinding(false);

                // 목표에 도달하지는 못 했지만 작업 자체는 끝났다는 의미이다.
                return true;
            }
            
            // 작업 노드는 OpenSet에서 빠져나오며 ClosedSet에 들어간다.
            processingNode->isInOpenSet   = false;
            processingNode->isInClosedSet = true;

            // 경로 설정
            std::vector<Vec2D<i32>> pathPoints = RetracePath(*processingNode);
            pathFindingRecord.TakePathPoints(std::move(pathPoints));
            
            // Processing
            if (processingNode->parent == nullptr) // 시작 노드
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
            }
            else // 파생 노드
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
            }

            // 목표 도달
            if (processingNode->x == destPos.x && processingNode->y == destPos.y)
            {
                pathFindingContext->RecordPathFinding(&pathFindingRecord);

                pathFindingContext->FinishPathFinding(true);

                return true;
            }

            // PathFindingNode가 아닌 Jump Points나 Wall과의 거리가 기입된 노드이다.
            const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

            Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

            // 목표 지점까지 수직 + 수평 거리
            i32 destDx = abs(destPos.x - processingPos.x);
            i32 destDy = abs(destPos.y - processingPos.y);

            // 인접 노드를 조회하며 OpenSet 갱신
            // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
            for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
            {
                PathFindingNode* nextNode = nullptr;
                f32 distance;

                if (IsDirectionCardinal(adjDir) == true && // 상하좌우
                    IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // 정확한 방향에 목표 지점 존재
                    std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // 직선 거리 이내에 목표 존재
                {
                    // 원문 : Goal is closer than wall distance or closer than or equal to jump point distance.
                    // 점프 해야 하는 위치가 목표 지점이다.
                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(destPos.x, destPos.y);
                    distance = processingNode->g + std::max(abs(destDx), abs(destDy));
                }
                else if (IsDirectionDiagonal(adjDir) == true && // 대각선
                         IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // 사분면 안에 목표 지점 존재
                         (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // 가로 거리 이내에 목표가 있거나
                          destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // 세로 거리 이내에 목표가 존재
                {
                    // 원문 : Goal is closer or equal in either row or column than wall or jump point distance.
                    // 가로 혹은 세로 거리 이내에 목표가 있다면 환승 지점(Target Jump Point)을 구성한다.
                    i32        minDiff = std::min(destDx, destDy); // 한 번 꺾어서 도달할 것이기 때문에 가로 거리와 세로 거리 중 값이 작은 것은 선택함.
                    Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir 방향에 minDiff를 곱하여 움직일 거리를 계산함.

                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(processingPos.x + toward.x, processingPos.y + toward.y);
                    distance = processingNode->g + (kGridDiagonalCost * minDiff);
                }
                else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
                {
                    // 해당 방향에 Jump Point가 있으면 JP가 있는 위치로 이동한다(벽은 음수로 평가되기에 안 들어 옮).
                    Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir 방향에 JP와의 거리를 곱하여 움직일 거리를 계산함.

                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(processingPos.x + toward.x, processingPos.y + toward.y);
                    distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                    // 기본 방위 방향은 단위가 1이지만 대각선은 따로 처리해야 한다.
                    if (IsDirectionDiagonal(adjDir) == true)
                    {
                        distance *= kGridDiagonalCost;
                    }

                    distance += processingNode->g;
                }

                // 다음에 이동할 Jump Point를 찾은 상태
                if (nextNode != nullptr)
                {
                    // 이미 사용된 노드
                    if (nextNode->isInClosedSet == true)
                        continue;

                    if (nextNode->isInOpenSet == false)
                    {
                        // 새로운 노드
                        nextNode->parent = processingNode;

                        // nextNode->g = distance;
                        // nextNode->h = heuristic;
                        // nextNode->f = distance + heuristic;

                        nextNode->g = distance;
                        nextNode->h = CalculateHeuristicCost(pathFindingContext->GetHeuristicType(), (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), pathFindingContext->GetHeuristicWeight());
                        nextNode->f = distance + nextNode->h;

                        nextNode->indegreeDirection = adjDir;

                        nextNode->isInOpenSet = true;

                        // OpenSet에 추가
                        pathFindingContext->EnqueuePathFindingNode(*nextNode);

                        pathFindingRecord.AddVisitedNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                        pathFindingRecord.AddOpenNewNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                    }
                    else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                    {
                        // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
                        nextNode->parent = processingNode;

                        // nextNode->g = distance;
                        // nextNode->f = distance + heuristic;

                        // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
                        nextNode->g = distance;
                        nextNode->f = distance + nextNode->h;

                        nextNode->indegreeDirection = adjDir;

                        // 갱신
                        pathFindingContext->UpdatePathFindingNode(*nextNode);

                        pathFindingRecord.AddVisitedNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                        pathFindingRecord.AddOpenUpdatedNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                    }
                }
            }

            pathFindingContext->RecordPathFinding(&pathFindingRecord);

            return true;
        }
    }
    
    return false;
}

bool JumpPointSearchPlusComplete(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    while (true)
    {
        PathFindingContext::Phase phase = pathFindingContext->GetPhase();

        if (PathFindingContext::Phase::Done == phase)
            break;

        if (JumpPointSearchPlusAdvance(pathFindingContext) == false)
            return false;
    }

    return true;
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
