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

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;

    // ��ġ�� �ش��ϴ� ��带 �������� ���� �ڷ���
    std::unordered_map<i32, PathFindingNode> posToPathFindingNodeMap;

    // posToPathFindingNodeMap���� ������ ��带 ó���� �켱���� ť�� ����� ���� OpenSet ���� ����
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;

    tls_PathFindingNodePQ.Clear();
    
    // ���� OpenSet�� ����
    PathFindingNode* startNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->f = 0.0f;

        startNode->isInOpenSet = true;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

    // ��ǥ�� ������ ������ ��ã�� ���� ����
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
    
        // �۾� ���� OpenSet���� ���������� ClosedSet�� ����.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
        
        // ��ǥ ����
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];

            // �̹� ���� ���
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // ���ο� ���
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet�� �߰�
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

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
    // ��ġ�� �ش��ϴ� ��带 �������� ���� �ڷ���
    static thread_local std::unordered_map<i32, PathFindingNode> tls_PosToPathFindingNodeMap;

    tls_PosToPathFindingNodeMap.clear();

    // tls_PosToPathFindingNodeMap���� ������ ��带 ó���� �켱���� ť�� ����� ���� OpenSet ���� ����
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;
    
    tls_PathFindingNodePQ.Clear();
    
    // ���� OpenSet�� ����
    PathFindingNode* startNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(startPos.x, startPos.y)];
    {
        startNode->x = startPos.x;
        startNode->y = startPos.y;

        startNode->f = 0.0f;

        startNode->isInOpenSet = true;
    }

    tls_PathFindingNodePQ.Enqueue(startNode);

    enqueuedCnt++;

    // ��ǥ�� ������ ������ ��ã�� ���� ����
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
    
        // �۾� ���� OpenSet���� ���������� ClosedSet�� ����.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
        
        // ��ǥ ����
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];

            // �̹� ���� ���
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // ���ο� ���
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet�� �߰�
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

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
    // ��ã�� ���̵� �߱ޱ�
    static thread_local ui32 pathFindingIdGenerator = 0;
    
    const ui32 kPathFindingId = pathFindingIdGenerator++;

    // ��ġ�� �ش��ϴ� ��带 �������� ���� �ڷ���
    static thread_local std::unordered_map<i32, PathFindingNode> tls_PosToPathFindingNodeMap;

    // ��ã�� ���̵� ������� ��带 �ĺ��ϸ� clear()�� ȣ������ �ʾƵ� �Ҵ�� ��带 ������ �� �ִ�.
    // tls_PosToPathFindingNodeMap.clear();

    // tls_PosToPathFindingNodeMap���� ������ ��带 ó���� �켱���� ť�� ����� ���� OpenSet ���� ����
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;
    
    tls_PathFindingNodePQ.Clear();

    // ���� OpenSet�� ����(���� �ʱ�ȭ ����)
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

    // ��ǥ�� ������ ������ ��ã�� ���� ����
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
    
        // �۾� ���� OpenSet���� ���������� ClosedSet�� ����.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
        
        // ��ǥ ����
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];
            
            // ��ã�� ���̵� ��ġ���� ������ ���ο� Ž������ �����Ѵ�.
            if (nextNode->pathFindingId != kPathFindingId)
            {
                nextNode->isInOpenSet   = false;
                nextNode->isInClosedSet = false;

                nextNode->pathFindingId = kPathFindingId;
            }

            // �̹� ���� ���
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // ���ο� ���
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet�� �߰�
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

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

    //
    i32 enqueuedCnt = 0;
    
    // ��ã�� ���̵� �߱ޱ�
    static thread_local ui32 pathFindingIdGenerator = 0;
    
    const ui32 kPathFindingId = pathFindingIdGenerator++;

    // ��ġ�� �ش��ϴ� ��带 �������� ���� �ڷ���
    static thread_local std::vector<PathFindingNode> tls_PathFindingNodes;
    
    // ��ã�� ���̵� ������� ��带 �ĺ��ϸ� clear()�� ȣ������ �ʾƵ� �Ҵ�� ��带 ������ �� �ִ�.
    // tls_PathFindingNodes.clear();
    
    // std::vector()�� resize()�� ���ο� ũ�Ⱑ ���� ũ�⺸�� Ŭ ��쿡�� ������ ��ģ��.
    tls_PathFindingNodes.resize(staticMap->GetSize());
    
    // tls_PathFindingNodes���� ������ ��带 ó���� �켱���� ť�� ����� ���� OpenSet ���� ����
    static thread_local PointerPriorityQueue<PathFindingNode*, PathFindingNodeComp> tls_PathFindingNodePQ;
    
    tls_PathFindingNodePQ.Clear();
    
    // ���� OpenSet�� ����(���� �ʱ�ȭ ����)
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

    // ��ǥ�� ������ ������ ��ã�� ���� ����
    while (tls_PathFindingNodePQ.Size() > 0)
    {
        PathFindingNode* processingNode;
        if (tls_PathFindingNodePQ.Dequeue(&processingNode) == false)
            return false;
    
        // �۾� ���� OpenSet���� ���������� ClosedSet�� ����.
        processingNode->isInOpenSet   = false;
        processingNode->isInClosedSet = true;
        
        // ��ǥ ����
        if (processingNode->x == destPos.x && processingNode->y == destPos.y)
        {
            if (outResult != nullptr)
            {
                *outResult = RetracePath(*processingNode);
            }
    
            return true;
        }

        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
        {
            PathFindingNode* nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(toAdjNode->x, toAdjNode->y)];
            
            // ��ã�� ���̵� ��ġ���� ������ ���ο� Ž������ �����Ѵ�.
            if (nextNode->pathFindingId != kPathFindingId)
            {
                nextNode->isInOpenSet   = false;
                nextNode->isInClosedSet = false;

                nextNode->pathFindingId = kPathFindingId;
            }

            // �̹� ���� ���
            if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                continue;
            
            // ���ο� ���
            {
                nextNode->x = toAdjNode->x;
                nextNode->y = toAdjNode->y;

                nextNode->parent = processingNode;

                nextNode->f = (f32)enqueuedCnt;

                nextNode->isInOpenSet = true;
            }

            // OpenSet�� �߰�
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
            // ���� OpenSet�� ����
            Vec2D<i32> startPos = pathFindingContext->GetStartPos();
            Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();

            PathFindingNode* startNode = pathFindingContext->GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
            {
                startNode->g = 0.0f;
                startNode->h = 0.0f;
                startNode->f = 0.0f;

                startNode->isInOpenSet = true;
            }

            // OpenList(OpenSet)�� �߰��ϴ� ���� ó���� ���̶� ���� �ʴ´�.
            pathFindingRecord.SetProcessingNode(-1, -1, -1, -1, 0.0f, 0.0f, 0.0f);

            pathFindingRecord.AddVisitedNode(startPos.x, startPos.y, startPos.x, startPos.y, 0.0f, 0.0f, 0.0f);
            pathFindingRecord.AddOpenNewNode(startPos.x, startPos.y, startPos.x, startPos.y, 0.0f, 0.0f, 0.0f);

            pathFindingContext->RecordPathFinding(&pathFindingRecord);

            pathFindingContext->StartPathFinding(*startNode);

            return true;
        }

        case PathFindingContext::Phase::PathFinding:
        {
            // ��ǥ�� ������ ������ ��ã�� ���� ����
            std::shared_ptr<StaticMap> staticMap = pathFindingContext->GetStaticMap();
            Vec2D<i32> destPos = pathFindingContext->GetDestinationPos();
            
            PathFindingNode* processingNode;
            if (pathFindingContext->DequeuePathFindingNode(&processingNode) == false)
            {
                // ��� ã�� ���������� ��ã�� �۾��� �����Ѵ�.
                pathFindingContext->FinishPathFinding(false);
            
                // ��ǥ�� ���������� �� ������ �۾� ��ü�� �����ٴ� �ǹ��̴�.
                return true;
            }
            
            // �۾� ���� OpenSet���� ���������� ClosedSet�� ����.
            processingNode->isInOpenSet   = false;
            processingNode->isInClosedSet = true;
            
            // ��� ����
            std::vector<Vec2D<i32>> pathPoints = RetracePath(*processingNode);
            pathFindingRecord.TakePathPoints(std::move(pathPoints));
            
            // Processing
            if (processingNode->parent == nullptr) // ���� ���
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, 0.0f, 0.0f, 0.0f);
            }
            else // �Ļ� ���
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->g, processingNode->g, 0.0f);
            }
            
            // ��ǥ ����
            if (processingNode->x == destPos.x && processingNode->y == destPos.y)
            {
                pathFindingContext->RecordPathFinding(&pathFindingRecord);

                pathFindingContext->FinishPathFinding(true);

                return true;
            }

            // �밢�� O : North -> East -> South -> West -> NorthWest -> NorthEast -> SouthEast -> SouthWest
            // �밢�� X : North -> East -> South -> West
            // ���� ��带 ��ȸ�ϸ� OpenSet ����
            for (auto [toAdjNode, toDir] : staticMap->GetAdjacentNodesAt(processingNode->x, processingNode->y))
            {
                PathFindingNode* nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(toAdjNode->x, toAdjNode->y);

                // �̹� ���� ���
                if (nextNode->isInClosedSet == true || nextNode->isInOpenSet == true)
                    continue;

                f32 distance = processingNode->g + ConvertToDistance(toDir);

                // ���ο� ���
                {
                    nextNode->parent = processingNode;

                    nextNode->g = distance; // �����ֱ� �뵵�� �켱���� ť���� ������ ��ġ�� ����.
                    nextNode->h = 0.0f;
                    nextNode->f = (f32)(pathFindingContext->GetTotalOpenSetEnqueuedCount());

                    nextNode->isInOpenSet = true;
                }

                // OpenSet�� �߰�
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
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
