// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Algorithm:JumpPointSearch;

import PointerPriorityQueue;

import PathFinding.Map;
import PathFinding.Context;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

/******************************************
*      JPS Helpers For SimpleContext      *
******************************************/

struct SimpleContext_UnorderedMap // std::unordered_map
{
    const StaticMap* staticMap = nullptr;
    
    Vec2D<i32> startPos;
    Vec2D<i32> destPos;

    HeuristicType heuristicType   = HeuristicType::Max;
    f32           heuristicWeight = 1.0f;

    // 위치를 바탕으로 노드를 가져온다.
    std::unordered_map<i32, PathFindingNode>* pathFindingNodeTable;

    // 비용이 저렴한 노드를 찾기 위한 우선순위 큐(OpenSet)
    PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp>* pathFindingNodePQ;

    bool usePathFindingId  = false;
    ui32 currPathFindingId = 0;

    PathFindingNode* GetOrCreatePathFindingNodeAt(i32 x, i32 y)
    {
        i32 nodeIdx = staticMap->ConvertToNodeIdx(x, y);

        // 위치를 Key로 하여 노드를 찾는다.
        PathFindingNode& pathFindingNode = (*pathFindingNodeTable)[nodeIdx];

        if (this->usePathFindingId == true && pathFindingNode.pathFindingId != this->currPathFindingId)
        {
            pathFindingNode.isInOpenSet   = false;
            pathFindingNode.isInClosedSet = false;

            pathFindingNode.pathFindingId = this->currPathFindingId;
        }

        return &pathFindingNode;
    }
};

struct SimpleContext_Vector // std::vector
{
    const StaticMap* staticMap = nullptr;
    
    Vec2D<i32> startPos;
    Vec2D<i32> destPos;

    HeuristicType heuristicType   = HeuristicType::Max;
    f32           heuristicWeight = 1.0f;

    // 위치를 바탕으로 노드를 가져온다.
    std::vector<PathFindingNode>* pathFindingNodeTable;

    // 비용이 저렴한 노드를 찾기 위한 우선순위 큐(OpenSet)
    PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp>* pathFindingNodePQ;

    ui32 currPathFindingId = 0;

    PathFindingNode* GetOrCreatePathFindingNodeAt(i32 x, i32 y)
    {
        i32 nodeIdx = staticMap->ConvertToNodeIdx(x, y);

        // 위치를 인덱스로 하여 노드를 찾는다.
        PathFindingNode& pathFindingNode = (*pathFindingNodeTable)[nodeIdx];
        
        if (pathFindingNode.pathFindingId != this->currPathFindingId)
        {
            pathFindingNode.isInOpenSet   = false;
            pathFindingNode.isInClosedSet = false;

            pathFindingNode.pathFindingId = this->currPathFindingId;
        }

        return &pathFindingNode;
    }
};

// OpenNode
template <typename SimpleContext>
bool SimpleContext_EnqueueOrUpdatePathNode(SimpleContext* simpleContext, 
                                           const Vec2D<i32>& nextPos, const Vec2D<i32>& processingPos,  f32 g, f32 h, i32 nextDirectionFlags);

// Cardinal
template <typename SimpleContext>
bool SimpleContext_SearchHorizontal(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dx, f32 g);

template <typename SimpleContext>
bool SimpleContext_SearchVertical(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dy, f32 g);

// Diagonal
template <typename SimpleContext>
bool SimpleContext_SearchDiagonal(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dx, i32 dy, f32 g);

/***************************
*      Implementation      *
***************************/

template <typename SimpleContext>
bool SimpleContext_EnqueueOrUpdatePathNode(SimpleContext* simpleContext,
                                           const Vec2D<i32>& nextPos, const Vec2D<i32>& processingPos, f32 g, f32 h, i32 nextDirectionFlags)
{
    /*
     * 아래 방식처럼 isInClosedSet과 isInOpenSet를 한 번에 처리하면 먼저 탐색한 쪽에서 코너를 돌 경우 결과가 이상해진다.
     * 이 경우에는 g 값은 비싸게 책정되기 때문에 다른 강제 이웃 탐지 방향에서 오는 g 값이 더 저렴한 경우가 생긴다.
     * 따라서 JPS라고 해도 estimated(f) 값을 갱신할 필요가 있다는 뜻이다.
     * 만약 Grid의 인덱스로부터 고유한 노드를 가져오는 방식이 이상하게 동작한다면 매번 노드 정보를 할당해서 우선순위 큐에 넣어야 한다.
     * !! 아직까지 발견된 버그는 없으니 Grid의 인덱스로부터 노드를 가져오는 방식을 취함. !!
     */
    // // 이미 사용된 노드
    // if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
    //     return false;
    // 
    // float heuristic = CalculateHeuristicCost(simpleContext->heuristicType, (f32)(simpleContext->destPos.x - nextNode->x), (f32)(simpleContext->destPos.x - nextNode.y), simpleContext->heuristicWeight);
    // 
    // // 새로운 노드
    // nextNode->x = nextPos.x;
    // nextNode->y = nextPos.y;
    // 
    // nextNode->parent = processingNode;
    // 
    // nextNode->g = g;
    // nextNode->h = heuristic;
    // nextNode->f = g + heuristic;
    // 
    // nextNode->nextDirectionFlags = nextDirectionFlags;
    // 
    // nextNode->isInOpenSet = true;
    // 
    // // OpenSet에 추가
    // simpleContext->tls_PathFindingNodePQ.Enqueue(nextNode);
    
    PathFindingNode* nextNode       = simpleContext->GetOrCreatePathFindingNodeAt(nextPos.x, nextPos.y);
    PathFindingNode* processingNode = simpleContext->GetOrCreatePathFindingNodeAt(processingPos.x, processingPos.y);

    // 함수로 기능을 옮겼다.
    // // 길찾기 아이디가 일치하지 않으면 새로운 탐색으로 간주한다.
    // if (simpleContext->usePathFindingId == true && nextNode->pathFindingId != simpleContext->currPathFindingId)
    // {
    //     nextNode->isInOpenSet   = false;
    //     nextNode->isInClosedSet = false;
    // 
    //     nextNode->pathFindingId = simpleContext->currPathFindingId;
    // }

    // 이미 사용된 노드
    if (nextNode->isInClosedSet == true)
        return false;

    f32 estimated = g + h;
    
    if (nextNode->isInOpenSet == false)
    {
        // 새로운 노드
        nextNode->x = nextPos.x;
        nextNode->y = nextPos.y;

        nextNode->parent = processingNode;
    
        nextNode->g = g;
        nextNode->h = h;
        nextNode->f = estimated;

        nextNode->nextDirectionFlags = nextDirectionFlags;

        nextNode->isInOpenSet = true;
    
        // OpenSet에 추가
        (*simpleContext->pathFindingNodePQ).Enqueue(nextNode);
    }
    else if (nextNode->f > estimated /*&& nextNode->isInOpenSet == true */)
    {
        // 이미 OpenSet에 들어가 있는 노드라면 비용이 저렴한 것으로 정보 갱신
        nextNode->parent = processingNode;
    
        nextNode->g = g;
        nextNode->f = estimated;

        nextNode->nextDirectionFlags = nextDirectionFlags;

        // 갱신
        (*simpleContext->pathFindingNodePQ).UpdateElement(nextNode);
    }

    return true;
}

template <typename SimpleContext>
bool SimpleContext_SearchHorizontal(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dx, f32 g)
{
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = simpleContext->destPos;

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;

    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, 0) == false)
            return false;
        
        // 탐색 정보 갱신
        herePos.x += dx;
        
        // PathFindingNode* hereNode = &(*simpleContext->pathFindingNodeTable)[simpleContext->staticMap->ConvertToNodeIdx(herePos.x, herePos.y)];
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(simpleContext->heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), simpleContext->heuristicWeight);
        g += kGridCardinalCost;
        
        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 도착 지점에 OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }
        
        // 위가 막힌 상황
        // 따라서 왼쪽 상단(↖) or 오른쪽 상단(↗)으로 방향을 꺾을 수 있다.
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, -1) == false && // 위가 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, -1) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, -1);
        }

        // 아래가 막힌 상황
        // 따라서 왼쪽 하단(↙) or 오른쪽 하단(↘)으로 방향을 꺾을 수 있다.
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, 1) == false && // 아래가 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, 1) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, 1);
        }
        
        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = simpleContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, 0);

            // OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

template <typename SimpleContext>
bool SimpleContext_SearchVertical(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dy, f32 g)
{
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = simpleContext->destPos;

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;

    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 0, dy) == false)
            return false;
        
        // 탐색 정보 갱신
        herePos.y += dy;

        // PathFindingNode* hereNode = &(*simpleContext->pathFindingNodeTable)[simpleContext->staticMap->ConvertToNodeIdx(herePos.x, herePos.y)];
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(simpleContext->heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), simpleContext->heuristicWeight);
        g += kGridCardinalCost;
        
        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 도착 지점에 OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }
        
        // 왼쪽이 막힌 상황
        // 따라서 왼쪽 상단(↖) or 왼쪽 하단(↙)으로 방향을 꺾을 수 있다.
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -1,  0) == false && // 왼쪽이 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -1, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(-1, dy);
        }

        // 오른쪽이 막힌 상황
        // 따라서 오른쪽 상단(↗) or 오른쪽 하단(↘)으로 방향을 꺾을 수 있다.
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 1,  0) == false && // 오른쪽이 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 1, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(1, dy);
        }
        
        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = simpleContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
            
            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(0, dy);

            // OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

template <typename SimpleContext>
bool SimpleContext_SearchDiagonal(SimpleContext* simpleContext, const Vec2D<i32>& processingPos, i32 dx, i32 dy, f32 g)
{
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = simpleContext->destPos;

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;

    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, dy) == false)
            return false;
        
        // 탐색 정보 갱신
        herePos.x += dx;
        herePos.y += dy;

        // PathFindingNode* hereNode = &(*simpleContext->pathFindingNodeTable)[simpleContext->staticMap->ConvertToNodeIdx(herePos.x, herePos.y)];
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(simpleContext->heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), simpleContext->heuristicWeight);
        g += kGridDiagonalCost;
        
        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 도착 지점에 OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }
        
        /**
         * 대각선 모퉁이 검사 1
         *
         * ◎ : herePos
         * 
         * ⬜◎⬜  ⬜◎⬜  ↘⬛↗   ↖⬛↙     l
         * ↗⬛↘  ↙⬛↖  ⬜◎⬜   ⬜◎⬜     l
         */
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, -dy) == false && // 위나 아래가 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, -dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, -dy);
        }

        /**
         * 대각선 모퉁이 검사 2
         *
         * ◎ : herePos
         * 
         * ↘⬜   ↖⬜   ⬜↗   ⬜↙           l
         * ⬛◎   ⬛◎   ◎⬛  ◎⬛           -l
         * ↙⬜   ↗⬜   ⬜↖   ⬜↘           l
         */
        if (simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -dx,  0) == false && // 왼쪽이나 오른쪽이 막힌 상태
            simpleContext->staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -dx, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(-dx, dy);
        }
        
        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = simpleContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
            
            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx,  0); // 수평
            nextDirectionFlags |= ConvertToJpsDirectionFlags( 0, dy); // 수직
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy); // 대각선

            // OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, nextDirectionFlags);

            return ret;
        }
        
        /**
         * 대각선 모퉁이를 발견하지 못 했으면 수직 + 수평 방향을 탐색해야 한다.
         * 수직 + 수평 방향을 탐색할 때 대각선의 분해 방향을 위임하는 것이 핵심이며,
         * 이를 통해 강제 이웃이 발생하는 지점을 발견할 수 있다.
         */

        // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
        PathFindingNode* hereNode = simpleContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
        
        // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        if (hereNode->isInClosedSet == true)
            return false;

        // 수평 방향 탐색
        if (SimpleContext_SearchHorizontal(simpleContext, herePos, dx, g) == true)
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy);
        }
        
        // 수직 방향 탐색
        if (SimpleContext_SearchVertical(simpleContext, herePos, dy, g) == true)
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy);
        }

        // 수직 + 수평 방향에 OpenNode가 생성되는 지점 발견
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // OpenNode 생성
            bool ret = SimpleContext_EnqueueOrUpdatePathNode(simpleContext, herePos, processingPos, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

/***********************************************
*      JPS Helpers For PathFindingContext      *
***********************************************/

// OpenNode
bool PathFindingContext_EnqueueOrUpdatePathNode(std::shared_ptr<PathFindingContext>& pathFindingContext, PathFindingRecord* pathFindingRecord,
                                                PathFindingNode* nextNode, const PathFindingNode& processingNode, f32 g, f32 h, i32 nextDirectionFlags);

// Cardinal
bool PathFindingContext_SearchHorizontal(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                         const PathFindingNode& processingNode, i32 dx, f32 g);

bool PathFindingContext_SearchVertical(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                       const PathFindingNode& processingNode, i32 dy, f32 g);

// Diagonal
bool PathFindingContext_SearchDiagonal(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                       const PathFindingNode& processingNode, i32 dx, i32 dy, f32 g);

/***************************
*      Implementation      *
***************************/

bool PathFindingContext_EnqueueOrUpdatePathNode(std::shared_ptr<PathFindingContext>& pathFindingContext, PathFindingRecord* pathFindingRecord,
                                                PathFindingNode* nextNode, const PathFindingNode& processingNode, f32 g, f32 h, i32 nextDirectionFlags)
{
    // 이미 사용된 노드
    if (nextNode->isInClosedSet == true)
        return false;

    Vec2D<i32> destPos   = pathFindingContext->GetDestinationPos();
    f32        estimated = g + h;

    if (nextNode->isInOpenSet == false)
    {
        // 새로운 노드
        nextNode->parent = &processingNode;

        nextNode->g = g;
        nextNode->h = h;
        nextNode->f = estimated;

        nextNode->nextDirectionFlags = nextDirectionFlags;

        nextNode->isInOpenSet = true;

        // OpenSet에 추가
        pathFindingContext->EnqueuePathFindingNode(*nextNode);

        // 시작 위치 기록
        pathFindingRecord->AddOpenNewNode(nextNode->x, nextNode->y, processingNode.x, processingNode.y, nextNode->f, nextNode->g, nextNode->h);
    }
    else if (nextNode->f > estimated /*&& nextNode->isInOpenSet == true */)
    {
        nextNode->parent = &processingNode;

        // 실제로 테스트해봤을 때 코너를 돌며 해당 위치를 OpenSet에 넣는 경우가 있다.
        // 이 경우에는 g 값은 비싸게 책정되기 때문에 다른 강제 이웃 탐지 방향에서 오는 g 값이 더 저렴한 경우가 생긴다.
        // 따라서 g 값을 갱신해야 한다.
        // !! 경로를 찾지 못 하는 현상이 발생하면 그때는 인덱스를 기반으로 노드를 가져오는 방식이 아닌 매번 새로 생성하는 방식을 택해야 함. !!
        // !! 아직까지는 이로 인한 버그가 발생한 적은 없으니 이대로 가도록 함. !!

        // OpenSet에 속한 노드는 이미 Heuristic 값이 계산된 상태
        nextNode->g = g;
        nextNode->f = estimated;

        nextNode->nextDirectionFlags = nextDirectionFlags;

        // 갱신
        pathFindingContext->UpdatePathFindingNode(*nextNode);

        // 갱신 내역 기록
        pathFindingRecord->AddOpenUpdatedNode(nextNode->x, nextNode->y, processingNode.x, processingNode.y, nextNode->f, nextNode->g, nextNode->h);
    }

    return true;
}

bool PathFindingContext_SearchHorizontal(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                         const PathFindingNode& processingNode, i32 dx, f32 g)
{
    Vec2D<i32> processingPos = { processingNode.x, processingNode.y };
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

    HeuristicType heuristicType   = pathFindingContext->GetHeuristicType();
    f32           heuristicWeight = pathFindingContext->GetHeuristicWeight();

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;

    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, 0) == false)
            return false;

        // 탐색 정보 갱신
        herePos.x += dx;

        // PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), heuristicWeight);
        g += kGridCardinalCost;

        // 기록
        pathFindingRecord->AddVisitedNode(herePos.x, herePos.y, processingPos.x, processingPos.y, g + heuristic, g, heuristic);

        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 여기서 처리하지 않는다(해당 위치의 OpenNode를 꺼내는 쪽에서 길찾기를 끝내는 것이 자연스러움).
            // pathFindingContext->FinishPathFinding(true);

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }

        // 위가 막힌 상황
        // 따라서 왼쪽 상단(↖) or 오른쪽 상단(↗)으로 방향을 꺾을 수 있다.
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, -1) == false && // 위가 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, -1) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, -1);
        }

        // 아래가 막힌 상황
        // 따라서 왼쪽 하단(↙) or 오른쪽 하단(↘)으로 방향을 꺾을 수 있다.
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, 1) == false && // 아래가 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, 1) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, 1);
        }

        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
            
            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, 0);

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

bool PathFindingContext_SearchVertical(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                       const PathFindingNode& processingNode, i32 dy, f32 g)
{
    Vec2D<i32> processingPos = { processingNode.x, processingNode.y };
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

    HeuristicType heuristicType   = pathFindingContext->GetHeuristicType();
    f32           heuristicWeight = pathFindingContext->GetHeuristicWeight();

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;

    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 0, dy) == false)
            return false;

        // 탐색 정보 갱신
        herePos.y += dy;

        // PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), heuristicWeight);
        g += kGridCardinalCost;

        // 기록
        pathFindingRecord->AddVisitedNode(herePos.x, herePos.y, processingPos.x, processingPos.y, g + heuristic, g, heuristic);

        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 여기서 처리하지 않는다(해당 위치의 OpenNode를 꺼내는 쪽에서 길찾기를 끝내는 것이 자연스러움).
            // pathFindingContext->FinishPathFinding(true);

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }

        // 왼쪽이 막힌 상황
        // 따라서 왼쪽 상단(↖) or 왼쪽 하단(↙)으로 방향을 꺾을 수 있다.
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -1,  0) == false && // 왼쪽이 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -1, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(-1, dy);
        }

        // 오른쪽이 막힌 상황
        // 따라서 오른쪽 상단(↗) or 오른쪽 하단(↘)으로 방향을 꺾을 수 있다.
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 1,  0) == false && // 오른쪽이 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, 1, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(1, dy);
        }

        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(0, dy);

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

bool PathFindingContext_SearchDiagonal(std::shared_ptr<PathFindingContext>& pathFindingContext, std::shared_ptr<StaticMap>& staticMap, PathFindingRecord* pathFindingRecord,
                                       const PathFindingNode& processingNode, i32 dx, i32 dy, f32 g)
{
    Vec2D<i32> processingPos = { processingNode.x, processingNode.y };
    Vec2D<i32> herePos = processingPos;
    Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

    HeuristicType heuristicType   = pathFindingContext->GetHeuristicType();
    f32           heuristicWeight = pathFindingContext->GetHeuristicWeight();

    i32 nextDirectionFlags = (i32)JpsDirectionFlags::None;
    
    while (true)
    {
        // 다음 지점으로 움직일 수 없을 때까지 탐색 진행
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, dy) == false)
            return false;
        
        // 탐색 정보 갱신
        herePos.x += dx;
        herePos.y += dy;

        // PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);
        // 
        // // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        // if (hereNode->isInClosedSet == true)
        //     return false;

        // 비용 갱신
        f32 heuristic = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - herePos.x), (f32)(destPos.y - herePos.y), heuristicWeight);
        g += kGridDiagonalCost;

        // 기록
        pathFindingRecord->AddVisitedNode(herePos.x, herePos.y, processingPos.x, processingPos.y, g + heuristic, g, heuristic);
        
        // 목표 도달
        if (herePos.x == destPos.x && herePos.y == destPos.y)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 여기서 처리하지 않는다(해당 위치의 OpenNode를 꺼내는 쪽에서 길찾기를 끝내는 것이 자연스러움).
            // pathFindingContext->FinishPathFinding(true);

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, (i32)JpsDirectionFlags::None);

            return ret;
        }

        /**
         * 대각선 모퉁이 검사 1
         *
         * ◎ : herePos
         * 
         * ⬜◎⬜  ⬜◎⬜  ↘⬛↗   ↖⬛↙     l
         * ↗⬛↘  ↙⬛↖  ⬜◎⬜   ⬜◎⬜     l
         */
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y,  0, -dy) == false && // 위나 아래가 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, dx, -dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, -dy);
        }

        /**
         * 대각선 모퉁이 검사 2
         *
         * ◎ : herePos
         * 
         * ↘⬜   ↖⬜   ⬜↗   ⬜↙           l
         * ⬛◎   ⬛◎   ◎⬛  ◎⬛           -l
         * ↙⬜   ↗⬜   ⬜↖   ⬜↘           l
         */
        if (staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -dx,  0) == false && // 왼쪽이나 오른쪽이 막힌 상태
            staticMap->CanMoveByDeltaPos(herePos.x, herePos.y, -dx, dy) == true)    // 여기서 대각선 방향으로 꺾을 수 있음.
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(-dx, dy);
        }

        // 코너 쪽에서 강제 이웃이 발견된 상태
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
            PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

            // 해당 위치가 이미 탐색된 상태면 빠져나온다.
            // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
            if (hereNode->isInClosedSet == true)
                return false;

            // 이동 방향 계승
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx,  0); // 수평
            nextDirectionFlags |= ConvertToJpsDirectionFlags( 0, dy); // 수직
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy); // 대각선

            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, nextDirectionFlags);

            return ret;
        }

        /**
         * 대각선 모퉁이를 발견하지 못 했으면 수직 + 수평 방향을 탐색해야 한다.
         * 수직 + 수평 방향을 탐색할 때 대각선의 분해 방향을 위임하는 것이 핵심이며,
         * 이를 통해 강제 이웃이 발생하는 지점을 발견할 수 있다.
         */

         // 정말 필요할 때만 꺼내서 검사해야 캐시 미스가 날 확률이 줄어든다(성능 차이 대략 2배 정도 남).
        PathFindingNode* hereNode = pathFindingContext->GetOrCreatePathFindingNodeAt(herePos.x, herePos.y);

        // 해당 위치가 이미 탐색된 상태면 빠져나온다.
        // 이를 허용하는 것은 맵을 빙 둘러서 탐색하겠다는 것과 같은 의미이기 때문에 생략하는 것이 좋다.
        if (hereNode->isInClosedSet == true)
            return false;

        // 수평 방향 탐색
        if (PathFindingContext_SearchHorizontal(pathFindingContext, staticMap, pathFindingRecord, *hereNode, dx, g) == true)
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy);
        }
        
        // 수직 방향 탐색
        if (PathFindingContext_SearchVertical(pathFindingContext, staticMap, pathFindingRecord, *hereNode, dy, g) == true)
        {
            nextDirectionFlags |= ConvertToJpsDirectionFlags(dx, dy);
        }

        // 수직 + 수평 방향에 OpenNode가 생성되는 지점 발견
        if (nextDirectionFlags |= (i32)JpsDirectionFlags::None)
        {
            // OpenNode 생성
            bool ret = PathFindingContext_EnqueueOrUpdatePathNode(pathFindingContext, pathFindingRecord, hereNode, processingNode, g, heuristic, nextDirectionFlags);

            return ret;
        }
    }

    return false;
}

/********************************
*      Main Implementation      *
********************************/

bool JumpPointSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;
    
    // JPS는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
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

    // 간이 Context 생성
    SimpleContext_UnorderedMap simpleContext;
    {
        simpleContext.staticMap = staticMap.get();

        simpleContext.startPos = startPos;
        simpleContext.destPos  = destPos;

        simpleContext.heuristicType   = heuristicType;
        simpleContext.heuristicWeight = heuristicWeight;

        simpleContext.pathFindingNodeTable = &posToPathFindingNodeMap;

        simpleContext.pathFindingNodePQ = &tls_PathFindingNodePQ;
    }
    
    // 최초 OpenSet에 삽입
    PathFindingNode* startNode = simpleContext.GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->h;

        // 최초 시작 노드는 모든 방향을 탐색한다.
        startNode->nextDirectionFlags = (i32)JpsDirectionFlags::AllFlags;

        startNode->isInOpenSet = true;
    }

    (*simpleContext.pathFindingNodePQ).Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while ((*simpleContext.pathFindingNodePQ).Size() > 0)
    {
        PathFindingNode* processingNode;
        if ((*simpleContext.pathFindingNodePQ).Dequeue(&processingNode) == false)
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
    
        // 기본 방위 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::North)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::East)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::South)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::West)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        // 대각선 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, 1, processingNode->g);
        }
    }

    return true;
}

bool JumpPointSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;
    
    // JPS는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
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

    // 간이 Context 생성
    SimpleContext_UnorderedMap simpleContext;
    {
        simpleContext.staticMap = staticMap.get();

        simpleContext.startPos = startPos;
        simpleContext.destPos  = destPos;

        simpleContext.heuristicType   = heuristicType;
        simpleContext.heuristicWeight = heuristicWeight;

        simpleContext.pathFindingNodeTable = &tls_PosToPathFindingNodeMap;

        simpleContext.pathFindingNodePQ = &tls_PathFindingNodePQ;
    }
    
    // 최초 OpenSet에 삽입
    PathFindingNode* startNode = simpleContext.GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->h;

        // 최초 시작 노드는 모든 방향을 탐색한다.
        startNode->nextDirectionFlags = (i32)JpsDirectionFlags::AllFlags;

        startNode->isInOpenSet = true;
    }

    (*simpleContext.pathFindingNodePQ).Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while ((*simpleContext.pathFindingNodePQ).Size() > 0)
    {
        PathFindingNode* processingNode;
        if ((*simpleContext.pathFindingNodePQ).Dequeue(&processingNode) == false)
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
    
        // 기본 방위 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::North)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::East)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::South)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::West)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        // 대각선 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, 1, processingNode->g);
        }
    }

    return true;
}

bool JumpPointSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;
    
    // JPS는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
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
    
    // 간이 Context 생성
    SimpleContext_UnorderedMap simpleContext;
    {
        simpleContext.staticMap = staticMap.get();

        simpleContext.startPos = startPos;
        simpleContext.destPos  = destPos;

        simpleContext.heuristicType   = heuristicType;
        simpleContext.heuristicWeight = heuristicWeight;

        simpleContext.pathFindingNodeTable = &tls_PosToPathFindingNodeMap;

        simpleContext.pathFindingNodePQ = &tls_PathFindingNodePQ;

        simpleContext.usePathFindingId  = true;
        simpleContext.currPathFindingId = kPathFindingId;
    }

    // 최초 OpenSet에 삽입(완전 초기화 진행)
    PathFindingNode* startNode = simpleContext.GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->h;

        // 최초 시작 노드는 모든 방향을 탐색한다.
        startNode->nextDirectionFlags = (i32)JpsDirectionFlags::AllFlags;
        
        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }

    (*simpleContext.pathFindingNodePQ).Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while ((*simpleContext.pathFindingNodePQ).Size() > 0)
    {
        PathFindingNode* processingNode;
        if ((*simpleContext.pathFindingNodePQ).Dequeue(&processingNode) == false)
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
    
        // 기본 방위 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::North)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::East)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::South)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::West)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        // 대각선 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, 1, processingNode->g);
        }
    }

    return true;
}

bool JumpPointSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;
    
    // JPS는 대각선을 허용할 때만 동작한다.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
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

    // 간이 Context 생성
    SimpleContext_Vector simpleContext;
    {
        simpleContext.staticMap = staticMap.get();

        simpleContext.startPos = startPos;
        simpleContext.destPos  = destPos;

        simpleContext.heuristicType   = heuristicType;
        simpleContext.heuristicWeight = heuristicWeight;

        simpleContext.pathFindingNodeTable = &tls_PathFindingNodes;

        simpleContext.pathFindingNodePQ = &tls_PathFindingNodePQ;

        simpleContext.currPathFindingId = kPathFindingId;
    }

    // 최초 OpenSet에 삽입(완전 초기화 진행)
    PathFindingNode* startNode = simpleContext.GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->h;

        // 최초 시작 노드는 모든 방향을 탐색한다.
        startNode->nextDirectionFlags = (i32)JpsDirectionFlags::AllFlags;
        
        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }

    (*simpleContext.pathFindingNodePQ).Enqueue(startNode);

    // 목표에 도달할 때까지 길찾기 로직 수행
    while ((*simpleContext.pathFindingNodePQ).Size() > 0)
    {
        PathFindingNode* processingNode;
        if ((*simpleContext.pathFindingNodePQ).Dequeue(&processingNode) == false)
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
    
        // 기본 방위 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::North)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::East)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::South)
        {
            SimpleContext_SearchVertical(&simpleContext, { processingNode->x, processingNode->y }, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::West)
        {
            SimpleContext_SearchHorizontal(&simpleContext, { processingNode->x, processingNode->y }, -1, processingNode->g);
        }

        // 대각선 방향
        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, -1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthEast)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, 1, 1, processingNode->g);
        }

        if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthWest)
        {
            SimpleContext_SearchDiagonal(&simpleContext, { processingNode->x, processingNode->y }, -1, 1, processingNode->g);
        }
    }

    return true;
}

bool JumpPointSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    if (pathFindingContext->GetAlgorithmType() != AlgorithmType::JumpPoint)
        return false;

    PathFindingContext::Phase phase = pathFindingContext->GetPhase();
    PathFindingRecord pathFindingRecord;

    std::shared_ptr<StaticMap> staticMap = pathFindingContext->GetStaticMap();
    Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

    switch (phase)
    {
        case PathFindingContext::Phase::Ready:
        {
            // JPS는 대각선을 허용할 때만 동작한다.
            if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
            {
                pathFindingContext->FinishPathFinding(false);

                return false;
            }

            // 최초 OpenSet에 삽입
            Vec2D<i32> startPos = pathFindingContext->GetStartPos();
            
            PathFindingNode* startNode = pathFindingContext->GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
            {
                startNode->g = 0.0f;
                startNode->h = CalculateHeuristicCost(pathFindingContext->GetHeuristicType(), (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), pathFindingContext->GetHeuristicWeight());
                startNode->f = startNode->h;

                // 최초 시작 노드는 모든 방향을 탐색한다.
                startNode->nextDirectionFlags = (i32)JpsDirectionFlags::AllFlags;

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

            // 기본 방위 방향
            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::North)
            {
                PathFindingContext_SearchVertical(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, -1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::East)
            {
                PathFindingContext_SearchHorizontal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, 1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::South)
            {
                PathFindingContext_SearchVertical(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, 1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::West)
            {
                PathFindingContext_SearchHorizontal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, -1, processingNode->g);
            }

            // 대각선 방향
            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthWest)
            {
                PathFindingContext_SearchDiagonal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, -1, -1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::NorthEast)
            {
                PathFindingContext_SearchDiagonal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, 1, -1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthEast)
            {
                PathFindingContext_SearchDiagonal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, 1, 1, processingNode->g);
            }

            if (processingNode->nextDirectionFlags & (i32)JpsDirectionFlags::SouthWest)
            {
                PathFindingContext_SearchDiagonal(pathFindingContext, staticMap, &pathFindingRecord, *processingNode, -1, 1, processingNode->g);
            }

            // 기록 내역 저장
            pathFindingContext->RecordPathFinding(&pathFindingRecord);

            return true;
        }
    }

    return false;
}

bool JumpPointSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    while (true)
    {
        PathFindingContext::Phase phase = pathFindingContext->GetPhase();

        if (PathFindingContext::Phase::Done == phase)
            break;

        if (JumpPointSearchAdvance(pathFindingContext) == false)
            return false;
    }

    return true;
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
