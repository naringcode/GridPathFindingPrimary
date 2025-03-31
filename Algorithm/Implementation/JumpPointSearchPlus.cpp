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
 * �� ��ũ�� ���� ���� ���� �ϸ� Flags�� ���� �б� ����� �ƴ� DirLookUpTable�� ������� �����Ѵ�.
 * ���⼭�� AdjacentDirection�� �̵��� ���⿡ ���� ���� �ε����� ����Ѵ�.
 * 
 * �÷��� ��ĵ� �Ұ����� ���� �ƴ����� �̸� ���̺�� �����Ϸ��� �迭�� 256(AllFlags + 1)��ŭ �Ҵ��ؾ� �Ѵ�.
 * �׸��� ��� Ž�� ���⿡ ���� ���� �ϳ��ϳ� ��������� �Ѵ�.
 * 
 * �� ��ũ������ ���� ���̺��� ������� Ž�������� �� �����ϸ� ���̺��� ������� �ʰ�
 * ���� ������ JPS�� �����ϰ� �÷��׷ε� ����� JPS+�� Ž�� ��θ� ������ �� �ִ�.
 * 
 * ������ JPS+�� �ٽ��� ���� �̿��� ��Ÿ�ӿ� ����ϴ� ���� �ƴ� ������ ����ϰ� �̸� ����ϴ� ���̱� ������ 
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
//     // None : �� ���� ��� ������ �����ϴµ� ����Ѵ�.
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

    // JPS+�� �밢���� ����� ���� �����Ѵ�.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+�� ���� ����ȭ�� �Ǿ� �־�� �Ѵ�.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;
    
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

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None�� ��� �������� ���εȴ�.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet = true;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

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

        // PathFindingNode�� �ƴ� Jump Points�� Wall���� �Ÿ��� ���Ե� ����̴�.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // ��ǥ �������� ���� + ���� �Ÿ�
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // �����¿�
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // ��Ȯ�� ���⿡ ��ǥ ���� ����
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // ���� �Ÿ� �̳��� ��ǥ ����
            {
                // ���� : Goal is closer than wall distance or closer than or equal to jump point distance.
                // ���� �ؾ� �ϴ� ��ġ�� ��ǥ �����̴�.
                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // �밢��
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // ��и� �ȿ� ��ǥ ���� ����
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // ���� �Ÿ� �̳��� ��ǥ�� �ְų�
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // ���� �Ÿ� �̳��� ��ǥ�� ����
            {
                // ���� : Goal is closer or equal in either row or column than wall or jump point distance.
                // ���� Ȥ�� ���� �Ÿ� �̳��� ��ǥ�� �ִٸ� ȯ�� ����(Target Jump Point)�� �����Ѵ�.
                i32        minDiff = std::min(destDx, destDy); // �� �� ��� ������ ���̱� ������ ���� �Ÿ��� ���� �Ÿ� �� ���� ���� ���� ������.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir ���⿡ minDiff�� ���Ͽ� ������ �Ÿ��� �����.
                
                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // �ش� ���⿡ Jump Point�� ������ JP�� �ִ� ��ġ�� �̵��Ѵ�(���� ������ �򰡵Ǳ⿡ �� ��� ��).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir ���⿡ JP���� �Ÿ��� ���Ͽ� ������ �Ÿ��� �����.

                nextNode = &posToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // �⺻ ���� ������ ������ 1������ �밢���� ���� ó���ؾ� �Ѵ�.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // ������ �̵��� Jump Point�� ã�� ����
            if (nextNode != nullptr)
            {
                // �̹� ���� ���
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // ���ο� ���
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet�� �߰�
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // �̹� OpenSet�� �� �ִ� ����� ����� ������ ������ ���� ����
                    nextNode->parent = processingNode;

                    // OpenSet�� ���� ���� �̹� Heuristic ���� ���� ����
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // ����
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
bool JumpPointSearchPlus2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+�� �밢���� ����� ���� �����Ѵ�.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+�� ���� ����ȭ�� �Ǿ� �־�� �Ѵ�.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

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

        startNode->parent = nullptr;

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None�� ��� �������� ���εȴ�.
        startNode->indegreeDirection = AdjacentDirection::None;
        
        startNode->isInOpenSet = true;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

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

        // PathFindingNode�� �ƴ� Jump Points�� Wall���� �Ÿ��� ���Ե� ����̴�.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // ��ǥ �������� ���� + ���� �Ÿ�
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // �����¿�
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // ��Ȯ�� ���⿡ ��ǥ ���� ����
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // ���� �Ÿ� �̳��� ��ǥ ����
            {
                // ���� : Goal is closer than wall distance or closer than or equal to jump point distance.
                // ���� �ؾ� �ϴ� ��ġ�� ��ǥ �����̴�.
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // �밢��
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // ��и� �ȿ� ��ǥ ���� ����
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // ���� �Ÿ� �̳��� ��ǥ�� �ְų�
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // ���� �Ÿ� �̳��� ��ǥ�� ����
            {
                // ���� : Goal is closer or equal in either row or column than wall or jump point distance.
                // ���� Ȥ�� ���� �Ÿ� �̳��� ��ǥ�� �ִٸ� ȯ�� ����(Target Jump Point)�� �����Ѵ�.
                i32        minDiff = std::min(destDx, destDy); // �� �� ��� ������ ���̱� ������ ���� �Ÿ��� ���� �Ÿ� �� ���� ���� ���� ������.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir ���⿡ minDiff�� ���Ͽ� ������ �Ÿ��� �����.
                
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // �ش� ���⿡ Jump Point�� ������ JP�� �ִ� ��ġ�� �̵��Ѵ�(���� ������ �򰡵Ǳ⿡ �� ��� ��).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir ���⿡ JP���� �Ÿ��� ���Ͽ� ������ �Ÿ��� �����.

                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // �⺻ ���� ������ ������ 1������ �밢���� ���� ó���ؾ� �Ѵ�.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // ������ �̵��� Jump Point�� ã�� ����
            if (nextNode != nullptr)
            {
                // �̹� ���� ���
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // ���ο� ���
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet�� �߰�
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // �̹� OpenSet�� �� �ִ� ����� ����� ������ ������ ���� ����
                    nextNode->parent = processingNode;

                    // OpenSet�� ���� ���� �̹� Heuristic ���� ���� ����
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // ����
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 ������ �뷫 20% ���� �� ������)
bool JumpPointSearchPlus3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+�� �밢���� ����� ���� �����Ѵ�.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+�� ���� ����ȭ�� �Ǿ� �־�� �Ѵ�.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;
    
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

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None�� ��� �������� ���εȴ�.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

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

        // PathFindingNode�� �ƴ� Jump Points�� Wall���� �Ÿ��� ���Ե� ����̴�.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // ��ǥ �������� ���� + ���� �Ÿ�
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // �����¿�
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // ��Ȯ�� ���⿡ ��ǥ ���� ����
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // ���� �Ÿ� �̳��� ��ǥ ����
            {
                // ���� : Goal is closer than wall distance or closer than or equal to jump point distance.
                // ���� �ؾ� �ϴ� ��ġ�� ��ǥ �����̴�.
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // �밢��
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // ��и� �ȿ� ��ǥ ���� ����
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // ���� �Ÿ� �̳��� ��ǥ�� �ְų�
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // ���� �Ÿ� �̳��� ��ǥ�� ����
            {
                // ���� : Goal is closer or equal in either row or column than wall or jump point distance.
                // ���� Ȥ�� ���� �Ÿ� �̳��� ��ǥ�� �ִٸ� ȯ�� ����(Target Jump Point)�� �����Ѵ�.
                i32        minDiff = std::min(destDx, destDy); // �� �� ��� ������ ���̱� ������ ���� �Ÿ��� ���� �Ÿ� �� ���� ���� ���� ������.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir ���⿡ minDiff�� ���Ͽ� ������ �Ÿ��� �����.
                
                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // �ش� ���⿡ Jump Point�� ������ JP�� �ִ� ��ġ�� �̵��Ѵ�(���� ������ �򰡵Ǳ⿡ �� ��� ��).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir ���⿡ JP���� �Ÿ��� ���Ͽ� ������ �Ÿ��� �����.

                nextNode = &tls_PosToPathFindingNodeMap[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // �⺻ ���� ������ ������ 1������ �밢���� ���� ó���ؾ� �Ѵ�.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // ������ �̵��� Jump Point�� ã�� ����
            if (nextNode != nullptr)
            {
                // ��ã�� ���̵� ��ġ���� ������ ���ο� Ž������ �����Ѵ�.
                if (nextNode->pathFindingId != kPathFindingId)
                {
                    nextNode->isInOpenSet   = false;
                    nextNode->isInClosedSet = false;

                    nextNode->pathFindingId = kPathFindingId;
                }

                // �̹� ���� ���
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // ���ο� ���
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet�� �߰�
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // �̹� OpenSet�� �� �ִ� ����� ����� ������ ������ ���� ����
                    nextNode->parent = processingNode;

                    // OpenSet�� ���� ���� �̹� Heuristic ���� ���� ����
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // ����
                    tls_PathFindingNodePQ.UpdateElement(nextNode);
                }
            }
        }
    }

    return true;
}

// ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
// 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ������ �� ����.
bool JumpPointSearchPlus4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult)
{
    if (staticMap == nullptr)
        return false;

    // JPS+�� �밢���� ����� ���� �����Ѵ�.
    if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
        return false;

    // JPS+�� ���� ����ȭ�� �Ǿ� �־�� �Ѵ�.
    if (false == staticMap->IsOptimizedForJpsPlus())
        return false;

    // ������� ������ ������ �߸� �����Ǿ� �ִ�.
    if (staticMap->IsWalkableNodeAt(startPos.x, startPos.y) == false ||
        staticMap->IsWalkableNodeAt(destPos.x, destPos.y)   == false)
        return false;

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

        startNode->g = 0.0f;
        startNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), heuristicWeight);
        startNode->f = startNode->g + startNode->h;

        // None�� ��� �������� ���εȴ�.
        startNode->indegreeDirection = AdjacentDirection::None;

        startNode->isInOpenSet   = true;
        startNode->isInClosedSet = false;

        startNode->pathFindingId = kPathFindingId;
    }
    
    tls_PathFindingNodePQ.Enqueue(startNode);

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

        // PathFindingNode�� �ƴ� Jump Points�� Wall���� �Ÿ��� ���Ե� ����̴�.
        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

        Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

        // ��ǥ �������� ���� + ���� �Ÿ�
        i32 destDx = abs(destPos.x - processingPos.x);
        i32 destDy = abs(destPos.y - processingPos.y);
        
        // ���� ��带 ��ȸ�ϸ� OpenSet ����
        // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
        for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
        {
            PathFindingNode* nextNode = nullptr;
            f32 distance;

            if (IsDirectionCardinal(adjDir) == true && // �����¿�
                IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // ��Ȯ�� ���⿡ ��ǥ ���� ����
                std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // ���� �Ÿ� �̳��� ��ǥ ����
            {
                // ���� : Goal is closer than wall distance or closer than or equal to jump point distance.
                // ���� �ؾ� �ϴ� ��ġ�� ��ǥ �����̴�.
                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(destPos.x, destPos.y)];
                {
                    nextNode->x = destPos.x;
                    nextNode->y = destPos.y;
                }

                distance = processingNode->g + std::max(abs(destDx), abs(destDy));
            }
            else if (IsDirectionDiagonal(adjDir) == true && // �밢��
                     IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // ��и� �ȿ� ��ǥ ���� ����
                     (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // ���� �Ÿ� �̳��� ��ǥ�� �ְų�
                      destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // ���� �Ÿ� �̳��� ��ǥ�� ����
            {
                // ���� : Goal is closer or equal in either row or column than wall or jump point distance.
                // ���� Ȥ�� ���� �Ÿ� �̳��� ��ǥ�� �ִٸ� ȯ�� ����(Target Jump Point)�� �����Ѵ�.
                i32        minDiff = std::min(destDx, destDy); // �� �� ��� ������ ���̱� ������ ���� �Ÿ��� ���� �Ÿ� �� ���� ���� ���� ������.
                Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir ���⿡ minDiff�� ���Ͽ� ������ �Ÿ��� �����.
                
                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = processingNode->g + (kGridDiagonalCost * minDiff);
            }
            else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
            {
                // �ش� ���⿡ Jump Point�� ������ JP�� �ִ� ��ġ�� �̵��Ѵ�(���� ������ �򰡵Ǳ⿡ �� ��� ��).
                Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir ���⿡ JP���� �Ÿ��� ���Ͽ� ������ �Ÿ��� �����.

                nextNode = &tls_PathFindingNodes[staticMap->ConvertToNodeIdx(processingPos.x + toward.x, processingPos.y + toward.y)];
                {
                    nextNode->x = processingPos.x + toward.x;
                    nextNode->y = processingPos.y + toward.y;
                }

                distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                // �⺻ ���� ������ ������ 1������ �밢���� ���� ó���ؾ� �Ѵ�.
                if (IsDirectionDiagonal(adjDir) == true)
                {
                    distance *= kGridDiagonalCost;
                }

                distance += processingNode->g;
            }

            // ������ �̵��� Jump Point�� ã�� ����
            if (nextNode != nullptr)
            {
                // ��ã�� ���̵� ��ġ���� ������ ���ο� Ž������ �����Ѵ�.
                if (nextNode->pathFindingId != kPathFindingId)
                {
                    nextNode->isInOpenSet   = false;
                    nextNode->isInClosedSet = false;

                    nextNode->pathFindingId = kPathFindingId;
                }

                // �̹� ���� ���
                if (nextNode->isInClosedSet == true)
                    continue;

                if (nextNode->isInOpenSet == false)
                {
                    // ���ο� ���
                    nextNode->parent = processingNode;

                    nextNode->g = distance;
                    nextNode->h = CalculateHeuristicCost(heuristicType, (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), heuristicWeight);
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    nextNode->isInOpenSet = true;

                    // OpenSet�� �߰�
                    tls_PathFindingNodePQ.Enqueue(nextNode);
                }
                else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                {
                    // �̹� OpenSet�� �� �ִ� ����� ����� ������ ������ ���� ����
                    nextNode->parent = processingNode;

                    // OpenSet�� ���� ���� �̹� Heuristic ���� ���� ����
                    nextNode->g = distance;
                    nextNode->f = distance + nextNode->h;

                    nextNode->indegreeDirection = adjDir;

                    // ����
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
            // JPS+�� �밢���� ����� ���� �����Ѵ�.
            if (staticMap->IsAllowedDiagonal() == false || staticMap->IsAllowedCorners() == false)
            {
                pathFindingContext->FinishPathFinding(false);

                return false;
            }

            // JPS+�� ���� ����ȭ�� �Ǿ� �־�� �Ѵ�.
            if (false == staticMap->IsOptimizedForJpsPlus())
            {
                pathFindingContext->FinishPathFinding(false);

                return false;
            }

            // ���� OpenSet�� ����
            Vec2D<i32> startPos = pathFindingContext->GetStartPos();
            Vec2D<i32> destPos  = pathFindingContext->GetDestinationPos();
            
            PathFindingNode* startNode = pathFindingContext->GetOrCreatePathFindingNodeAt(startPos.x, startPos.y);
            {
                startNode->g = 0.0f;
                startNode->h = CalculateHeuristicCost(pathFindingContext->GetHeuristicType(), (f32)(destPos.x - startPos.x), (f32)(destPos.y - startPos.y), pathFindingContext->GetHeuristicWeight());
                startNode->f = startNode->h;

                // None�� ��� �������� ���εȴ�.
                startNode->indegreeDirection = AdjacentDirection::None;

                startNode->isInOpenSet = true;
            }
            
            // OpenList(OpenSet)�� �߰��ϴ� ���� ó���� ���̶� ���� �ʴ´�.
            pathFindingRecord.SetProcessingNode(-1, -1, -1, -1, 0.0f, 0.0f, 0.0f);

            pathFindingRecord.AddVisitedNode(startPos.x, startPos.y, startPos.x, startPos.y, startNode->f, 0.0f, startNode->h);
            pathFindingRecord.AddOpenNewNode(startPos.x, startPos.y, startPos.x, startPos.y, startNode->f, 0.0f, startNode->h);

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
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->x, processingNode->y, processingNode->f, 0.0f, processingNode->h);
            }
            else // �Ļ� ���
            {
                pathFindingRecord.SetProcessingNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
            
                pathFindingRecord.AddVisitedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
                pathFindingRecord.AddClosedNode(processingNode->x, processingNode->y, processingNode->parent->x, processingNode->parent->y, processingNode->f, processingNode->g, processingNode->h);
            }

            // ��ǥ ����
            if (processingNode->x == destPos.x && processingNode->y == destPos.y)
            {
                pathFindingContext->RecordPathFinding(&pathFindingRecord);

                pathFindingContext->FinishPathFinding(true);

                return true;
            }

            // PathFindingNode�� �ƴ� Jump Points�� Wall���� �Ÿ��� ���Ե� ����̴�.
            const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(processingNode->x, processingNode->y);

            Vec2D<i32> processingPos{ processingNode->x, processingNode->y };

            // ��ǥ �������� ���� + ���� �Ÿ�
            i32 destDx = abs(destPos.x - processingPos.x);
            i32 destDy = abs(destPos.y - processingPos.y);

            // ���� ��带 ��ȸ�ϸ� OpenSet ����
            // for (const AdjacentDirection adjDir : g_DirLookUpTable[openNode.indegreeDirection])
            for (const AdjacentDirection adjDir : staticMap->GetJumpDirLookUpTable(processingNode->indegreeDirection))
            {
                PathFindingNode* nextNode = nullptr;
                f32 distance;

                if (IsDirectionCardinal(adjDir) == true && // �����¿�
                    IsDestinationInExactDirection(processingPos, destPos, adjDir) == true && // ��Ȯ�� ���⿡ ��ǥ ���� ����
                    std::max(destDx, destDy) <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])) // ���� �Ÿ� �̳��� ��ǥ ����
                {
                    // ���� : Goal is closer than wall distance or closer than or equal to jump point distance.
                    // ���� �ؾ� �ϴ� ��ġ�� ��ǥ �����̴�.
                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(destPos.x, destPos.y);
                    distance = processingNode->g + std::max(abs(destDx), abs(destDy));
                }
                else if (IsDirectionDiagonal(adjDir) == true && // �밢��
                         IsDestinationInGeneralDirection(processingPos, destPos, adjDir) == true && // ��и� �ȿ� ��ǥ ���� ����
                         (destDx <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir]) || // ���� �Ÿ� �̳��� ��ǥ�� �ְų�
                          destDy <= abs(jumpPointNode.jumpDistanceTable[(i32)adjDir])))  // ���� �Ÿ� �̳��� ��ǥ�� ����
                {
                    // ���� : Goal is closer or equal in either row or column than wall or jump point distance.
                    // ���� Ȥ�� ���� �Ÿ� �̳��� ��ǥ�� �ִٸ� ȯ�� ����(Target Jump Point)�� �����Ѵ�.
                    i32        minDiff = std::min(destDx, destDy); // �� �� ��� ������ ���̱� ������ ���� �Ÿ��� ���� �Ÿ� �� ���� ���� ���� ������.
                    Vec2D<i32> toward  = ConvertToDeltaPos(adjDir, minDiff); // adjDir ���⿡ minDiff�� ���Ͽ� ������ �Ÿ��� �����.

                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(processingPos.x + toward.x, processingPos.y + toward.y);
                    distance = processingNode->g + (kGridDiagonalCost * minDiff);
                }
                else if (jumpPointNode.jumpDistanceTable[(i32)adjDir] > 0)
                {
                    // �ش� ���⿡ Jump Point�� ������ JP�� �ִ� ��ġ�� �̵��Ѵ�(���� ������ �򰡵Ǳ⿡ �� ��� ��).
                    Vec2D<i32> toward = ConvertToDeltaPos(adjDir, jumpPointNode.jumpDistanceTable[(i32)adjDir]); // adjDir ���⿡ JP���� �Ÿ��� ���Ͽ� ������ �Ÿ��� �����.

                    nextNode = pathFindingContext->GetOrCreatePathFindingNodeAt(processingPos.x + toward.x, processingPos.y + toward.y);
                    distance = (f32)jumpPointNode.jumpDistanceTable[(i32)adjDir];

                    // �⺻ ���� ������ ������ 1������ �밢���� ���� ó���ؾ� �Ѵ�.
                    if (IsDirectionDiagonal(adjDir) == true)
                    {
                        distance *= kGridDiagonalCost;
                    }

                    distance += processingNode->g;
                }

                // ������ �̵��� Jump Point�� ã�� ����
                if (nextNode != nullptr)
                {
                    // �̹� ���� ���
                    if (nextNode->isInClosedSet == true)
                        continue;

                    if (nextNode->isInOpenSet == false)
                    {
                        // ���ο� ���
                        nextNode->parent = processingNode;

                        // nextNode->g = distance;
                        // nextNode->h = heuristic;
                        // nextNode->f = distance + heuristic;

                        nextNode->g = distance;
                        nextNode->h = CalculateHeuristicCost(pathFindingContext->GetHeuristicType(), (f32)(destPos.x - nextNode->x), (f32)(destPos.y - nextNode->y), pathFindingContext->GetHeuristicWeight());
                        nextNode->f = distance + nextNode->h;

                        nextNode->indegreeDirection = adjDir;

                        nextNode->isInOpenSet = true;

                        // OpenSet�� �߰�
                        pathFindingContext->EnqueuePathFindingNode(*nextNode);

                        pathFindingRecord.AddVisitedNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                        pathFindingRecord.AddOpenNewNode(nextNode->x, nextNode->y, processingNode->x, processingNode->y, nextNode->f, nextNode->g, nextNode->h);
                    }
                    else if (nextNode->g > distance /*&& nextNode->isInOpenSet == true */)
                    {
                        // �̹� OpenSet�� �� �ִ� ����� ����� ������ ������ ���� ����
                        nextNode->parent = processingNode;

                        // nextNode->g = distance;
                        // nextNode->f = distance + heuristic;

                        // OpenSet�� ���� ���� �̹� Heuristic ���� ���� ����
                        nextNode->g = distance;
                        nextNode->f = distance + nextNode->h;

                        nextNode->indegreeDirection = adjDir;

                        // ����
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
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
