// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Algorithm:BreadthFirstSearch;

import PointerPriorityQueue;

import PathFinding.Map;
import PathFinding.Context;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

bool BreadthFirstSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;

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

        startNode->f = 0.0f;

        startNode->isInOpenSet = true;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

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

        // 인접 노드를 조회하며 OpenSet 갱신
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];

            // 이미 사용된 노드
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // 새로운 노드
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet에 추가
            tls_PathFindingNodePQ.Enqueue(nextNode);

            enqueuedCnt++;
        }
    }

    return true;
}

bool BreadthFirstSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
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

        startNode->f = 0.0f;

        startNode->isInOpenSet = true;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

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

        // 인접 노드를 조회하며 OpenSet 갱신
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];

            // 이미 사용된 노드
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // 새로운 노드
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet에 추가
            tls_PathFindingNodePQ.Enqueue(nextNode);

            enqueuedCnt++;
        }
    }

    return true;
}

bool BreadthFirstSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
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

        startNode->f = 0.0f;
        
        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

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

        // 인접 노드를 조회하며 OpenSet 갱신
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];
            
            // 길찾기 아이디가 일치하지 않으면 새로운 탐색으로 간주한다.
            if (nextNode->pathFindingId != kPathFindingId)
            {
                nextNode->isInOpenSet   = false;
                nextNode->isInClosedSet = false;

                nextNode->pathFindingId = kPathFindingId;
            }

            // 이미 사용된 노드
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // 새로운 노드
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet에 추가
            tls_PathFindingNodePQ.Enqueue(nextNode);

            enqueuedCnt++;
        }
    }

    return true;
}

bool BreadthFirstSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // 출발지와 도착지 지점이 잘못 설정되어 있다.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
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

        startNode->f = 0.0f;
        
        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

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

        // 인접 노드를 조회하며 OpenSet 갱신
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];
            
            // 길찾기 아이디가 일치하지 않으면 새로운 탐색으로 간주한다.
            if (nextNode->pathFindingId != kPathFindingId)
            {
                nextNode->isInOpenSet   = false;
                nextNode->isInClosedSet = false;

                nextNode->pathFindingId = kPathFindingId;
            }

            // 이미 사용된 노드
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // 새로운 노드
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet에 추가
            tls_PathFindingNodePQ.Enqueue(nextNode);

            enqueuedCnt++;
        }
    }

    return true;
}

bool BreadthFirstSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    if (pathFindingContext->GetAlgorithmType() != AlgorithmType::BreadthFirst)
        return false;

    PathFindingContext::Phase phase = pathFindingContext->GetPhase();
    PathFindingRecord pathFindingRecord;

    switch (phase)
    {
        case PathFindingContext::Phase::Ready:
        {
            // 최초 OpenSet에 삽입
            Vec2D<i32> startPos = pathFindingContext->GetStartPos();
            Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

            PathFindingNode* startNode = pathFindingContext->GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
            {
                startNode->g = 0.0f;
                startNode->h = 0.0f;
                startNode->f = 0.0f;

                startNode->isInOpenSet = true;
            }

            // OpenList(OpenSet)에 추가하는 것은 처리한 것이라 보지 않는다.
            pathFindingRecord.SetProcessingNode(-1, -1, -1, -1, 0.0f, 0.0f, 0.0f);

            pathFindingRecord.AddVisitedNode(startPos.x, startPos.y, startPos.x, startPos.y, 0.0f, 0.0f, 0.0f);
            pathFindingRecord.AddOpenNewNode(startPos.x, startPos.y, startPos.x, startPos.y, 0.0f, 0.0f, 0.0f);

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
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
            }
            else // 파생 노드
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
            }
            
            // 목표 도달
            if (processingNode->x == destPos.x && processingNode->y == destPos.y)
            {
                pathFindingContext->RecordPathFinding(&pathFindingRecord);

                pathFindingContext->FinishPathFinding(true);

                return true;
            }

            // 대각선 O : North -> East -> South -> West -> NorthWest -> NorthEast -> SouthEast -> SouthWest
            // 대각선 X : North -> East -> South -> West
            // 인접 노드를 조회하며 OpenSet 갱신
            for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
            {
                PathFindingNode* nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(toAdjNode->x, toAdjNode->y);

                // 이미 사용된 노드
                if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                    continue;

                f32 distance = processingNode->g + ConvertToDistance(toDir);

                // 새로운 노드
                {
                    nextNode->parent = processingNode;

                    nextNode->g = distance; // 보여주기 용도로 우선순위 큐에는 영향을 미치지 않음.
                    nextNode->h = 0.0f;
                    nextNode->f = (f32)(pathFindingContext->GetTotalOpenSetEnqueuedCount());

                    nextNode->isInOpenSet = true;
                }

                // OpenSet에 추가
                pathFindingContext->EnqueuePathFindingNode(*nextNode);

                pathFindingRecord.AddVisitedNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->g, nextNode->g, 0.0f);
                pathFindingRecord.AddOpenNewNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->g, nextNode->g, 0.0f);
            }

            pathFindingContext->RecordPathFinding(&pathFindingRecord);

            return true;
        }
    }

    return false;
}

bool BreadthFirstSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext)
{
    if (pathFindingContext == nullptr)
        return false;

    while (true)
    {
        PathFindingContext::Phase phase = pathFindingContext->GetPhase();

        if (PathFindingContext::Phase::Done == phase)
            break;

        if (BreadthFirstSearchAdvance(pathFindingContext) == false)
            return false;
    }

    return true;
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
