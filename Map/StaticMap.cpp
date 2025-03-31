// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Map:StaticMap;

import Math;

import :DynamicMap;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

void StaticMap::Deleter::operator()(StaticMap* instance) const
{
    if (instance == nullptr)
        return;

    instance->_width = 0;
    instance->_height = 0;

    instance->_nodes.clear();

    //
    instance->_blockedPositions.clear();

    instance->_neighborTables.clear();
    instance->_adjNodeTables.clear();

    //
    instance->_jumpPointNodes.clear();

    std::ranges::for_each(instance->_jumpDirLookUpTables, [](auto& container) { container.clear(); });

    delete instance;
}

std::shared_ptr<StaticMap> StaticMap::Create(std::shared_ptr<DynamicMap>& dynamicMap, bool allowDiagonal, bool allowCorners)
{
    if (dynamicMap == nullptr)
        return nullptr;

    StaticMap* instance = new StaticMap{ };
    
    instance->_width  = dynamicMap->GetWidth();
    instance->_height = dynamicMap->GetHeight();

    instance->_isAllowedDiagonal = allowDiagonal;
    instance->_isAllowedCorners  = allowCorners;

    // ��� ����
    i32 mapSize = dynamicMap->GetSize();

    instance->_nodes.resize(mapSize);

    instance->_neighborTables.resize(mapSize);
    instance->_adjNodeTables.resize(mapSize);

    // Legacy Code
    // for (i32 y = 0; y < instance->_height; y++)
    // {
    //     for (i32 x = 0; x < instance->_width; x++)
    //     {
    //         i32 idx = x + (y * instance->_width);
    // 
    //         instance->_nodes[idx] = dynamicMap->GetNodeAt(x, y);
    // 
    //         //
    //         if (instance->_nodes[idx].state == NodeState::Blocked)
    //         {
    //             instance->_blockedPositions.push_back({ x, y });
    //         }
    // 
    //         instance->_adjNodeTables[idx].reserve(8);
    //     }
    // }

    std::ranges::for_each(std::views::iota(0, mapSize), [&](i32 idx) {
        i32 x = idx % instance->GetWidth();
        i32 y = idx / instance->GetWidth();

        instance->_nodes[idx] = dynamicMap->GetNodeAt(x, y);

        if (instance->_nodes[idx].state == NodeState::Blocked)
        {
            instance->_blockedPositions.push_back({ x, y });
        }

        instance->_adjNodeTables[idx].reserve(8);
    });

    // ����ȭ ����
    instance->optimize();

    return std::shared_ptr<StaticMap>{ instance, Deleter{ } };
}

std::shared_ptr<StaticMap> StaticMap::Clone() const
{
    StaticMap* instance = new StaticMap{ };
    
    instance->_width  = this->GetWidth();
    instance->_height = this->GetHeight();

    instance->_isAllowedDiagonal = this->_isAllowedDiagonal;
    instance->_isAllowedCorners  = this->_isAllowedCorners;

    // ��� ����
    i32 mapSize = this->GetSize();

    instance->_nodes.resize(mapSize);

    instance->_neighborTables.resize(mapSize);
    instance->_adjNodeTables.resize(mapSize);

    // Legacy Code
    // for (i32 y = 0; y < instance->_height; y++)
    // {
    //     for (i32 x = 0; x < instance->_width; x++)
    //     {
    //         i32 idx = x + (y * instance->_width);
    // 
    //         instance->_nodes[idx] = this->GetNodeAt(x, y);
    // 
    //         //
    //         if (instance->_nodes[idx].state == NodeState::Blocked)
    //         {
    //             instance->_blockedPositions.push_back({ x, y });
    //         }
    // 
    //         instance->_adjNodeTables[idx].reserve(8);
    //     }
    // }

    std::ranges::for_each(std::views::iota(0, mapSize), [&](i32 idx) {
        i32 x = idx % instance->GetWidth();
        i32 y = idx / instance->GetWidth();

        instance->_nodes[idx] = this->GetNodeAt(x, y);

        if (instance->_nodes[idx].state == NodeState::Blocked)
        {
            instance->_blockedPositions.push_back({ x, y });
        }

        instance->_adjNodeTables[idx].reserve(8);
    });

    // ����ȭ ����
    instance->optimize();

    if (this->IsOptimizedForJpsPlus() == true)
    {
        instance->OptimizeForJpsPlus();
    }

    return std::shared_ptr<StaticMap>{ instance, Deleter{ } };
}

bool StaticMap::OptimizeForJpsPlus()
{
    // �ڳ� ��ü�� ������� ������ JPS+ ���� ����ϴ� �������� �ڳ� ���θ� ��ȸ�Ѵ�(_isAllowedCorners�� true���� ��).
    if (_isAllowedDiagonal == false || _isAllowedCorners == false)
        return false;

    if (_isOptimizedForJpsPlus == false)
    {
        _isOptimizedForJpsPlus = true;

        _jumpPointNodes.resize(this->GetSize());

        this->prepareJumpLookUpTable();

        this->preprocessPrimaryJumpPoints();
        this->preprocessStraightJumpPoints();
        this->preprocessDiagonalJumpPoints();
    }

    return true;
}

bool StaticMap::HasLineOfSight_BresenhamLines(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const
{
    /*
     * Bresenham's Line References
     * https://www.youtube.com/watch?v=RGB-wlatStc
     * http://forum.falinux.com/zbxe/index.php?document_srl=406146
     * https://riptutorial.com/ko/algorithm/example/25012/bresenham-%EC%84%A0-%EA%B7%B8%EB%A6%AC%EA%B8%B0-%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98
     * https://jebae.github.io/2019/05/05/bresenham-line-algorithm/
     * https://kukuta.tistory.com/186
     */
    
    i32 x;
    i32 y;
    
    i32 endX;
    i32 endY;
    
    i32 dx = abs(destPos.x - startPos.x);
    i32 dy = abs(destPos.y - startPos.y);
    
    i32 dxDouble = 2 * dx;
    i32 dyDouble = 2 * dy;
    
    i32 errorBound;
    i32 increment = 1;
    
    // ���Ⱑ �ϸ��ϸ�(1���� ������) x++�� p���� üũ�ؼ� y�� �������Ѿ� �Ѵ�.
    if (dy < dx)
    {
        // x���� ó���� �����̱⿡ x���� ���� ���� �������� ��´�.
        if (startPos.x < destPos.x)
        {
            x = startPos.x;
            y = startPos.y;
    
            endX = destPos.x;
            endY = destPos.y;
        }
        else
        {
            x = destPos.x;
            y = destPos.y;
    
            endX = startPos.x;
            endY = startPos.y;
        }
    
        // dy�� �������� ��Ȳ(abs()�� �����Ű�� dy�� x�� ��Ī�� �Ǳ� ������ increment�� ��Ī���Ѿ� �Ѵ�.
        if (endY - y < 0)
        {
            increment = -1;
        }
    
        errorBound = dyDouble - dx; // p0 = 2dy - dx
    
        while (x <= endX)
        {
            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
                return false;
    
            x++;
    
            if (errorBound < 0)
            {
                errorBound = errorBound + dyDouble; // pnext = pk + 2dy
            }
            else
            {
                errorBound = errorBound + dyDouble - dxDouble; // pnext = pk + 2dy - 2dx
    
                y += increment;
            }
    
            // ���� ���Ŀ��� �̷��� �����
            // if (errorBound < 0)
            // {
            //     errorBound = errorBound + dyDouble; // pnext = 2dy + pk
            // }
            // else
            // {
            //     errorBound = errorBound + dyDouble - dxDouble; // pnext = 2dy - 2dx + pk
            // 
            //     y++;
            // }
        }
    }
    
    // ���Ⱑ ���ĸ���(1���� ũ��) y++�� p���� üũ�ؼ� x�� �������Ѿ� �Ѵ�.
    // y = x�� ��Ī�̱� ������ x�� y�� ���� ġȯ�ϸ� �ȴ�.
    else
    {
        // y���� ó���� �����̱⿡ y���� ���� ���� �������� ��´�.
        if (startPos.y < destPos.y)
        {
            x = startPos.x;
            y = startPos.y;
    
            endX = destPos.x;
            endY = destPos.y;
        }
        else
        {
            x = destPos.x;
            y = destPos.y;
    
            endX = startPos.x;
            endY = startPos.y;
        }
    
        // dx�� �������� ��Ȳ(abs()�� �����Ű�� dx�� y�� ��Ī�� �Ǳ� ������ increment�� ��Ī���Ѿ� �Ѵ�.
        if (endX - x < 0)
        {
            increment = -1;
        }
    
        errorBound = dxDouble - dy; // p0 = 2dx - dy
    
        while (y <= endY)
        {
            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
                return false;
    
            y++;
    
            if (errorBound < 0)
            {
                errorBound = errorBound + dxDouble; // pnext = pk + 2dx
            }
            else
            {
                errorBound = errorBound + dxDouble - dyDouble; // pnext = pk + 2dx - 2dy
    
                x += increment;
            }
    
            // ���� ���Ŀ��� �̷��� �����
            // if (errorBound < 0)
            // {
            //     errorBound = errorBound + dxDouble; // pnext = 2dx + pk
            // }
            // else
            // {
            //     errorBound = errorBound + dxDouble - dyDouble; // pnext = 2dx - 2dy + pk
            // 
            //     x++;
            // }
        }
    }

    return true;
}

bool PathFinding::StaticMap::HasLineOfSight_OrthogonalSteps(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const
{
    /**
     * �׽�Ʈ�غ��� Bresenham Line�� � Ÿ���� ����ϴ��� �������� �� �ϴ� ������ �߻��ߴ�.
     *
     * https://www.redblobgames.com/grids/line-drawing/
     * 
     * ���å
     * 1. Orthogonal steps
     * 2. Supercover lines
     */

    /*****************************
    *      Orthogonal Steps      *
    *****************************/
    i32 dx = destPos.x - startPos.x;
    i32 dy = destPos.y - startPos.y;
    
    i32 signX = Math::Sign(dx);
    i32 signY = Math::Sign(dy);
    
    i32 nx = abs(dx);
    i32 ny = abs(dy);
    
    i32 ix = 0;
    i32 iy = 0;
    
    Vec2D<i32> here{ startPos };
    
    while (ix < nx || iy < ny)
    {
        // if (((0.5f + (f32)ix) / (f32)nx) < ((0.5f + (f32)iy) / (f32)ny))
        // {
        //     // Next step is horizontal
        //     here.x += signX;
        //     ix++;
        // }

        // nx�� ny�� �Ǽ��̰� �� ���� 0���� ������ inf�� ���´�(���� �ǿ����ڰ� ����̱� ������ ���� ���Ѵ�� ����).
        i32 chkNx = (nx != 0) ? (nx * 10) : 1;
        i32 chkNy = (ny != 0) ? (ny * 10) : 1;

        // �纯�� 10�� ���� ���� nx�� ny�� �߰������� ���� ��
        if ((5 + ix * 10) * chkNy < (5 + iy * 10) * chkNx)
        {
            // Next step is horizontal
            here.x += signX;
            ix++;
        }
        else
        {
            // Next step is vertical
            here.y += signY;
            iy++;
        }
    
        if (IsWalkableNodeAt(here.x, here.y) == false)
            return false;
    }

    return true;
}

bool PathFinding::StaticMap::HasLineOfSight_SupercoverLines(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const
{
    /**
     * �׽�Ʈ�غ��� Bresenham Line�� � Ÿ���� ����ϴ��� �������� �� �ϴ� ������ �߻��ߴ�.
     *
     * https://www.redblobgames.com/grids/line-drawing/
     * 
     * ���å
     * 1. Orthogonal steps
     * 2. Supercover lines
     */

    /*****************************
    *      Supercover Lines      *
    *****************************/
    i32 dx = destPos.x - startPos.x;
    i32 dy = destPos.y - startPos.y;
    
    i32 signX = Math::Sign(dx);
    i32 signY = Math::Sign(dy);
    
    i32 nx = abs(dx);
    i32 ny = abs(dy);
    
    i32 ix = 0;
    i32 iy = 0;
    
    Vec2D<i32> here{ startPos };
    
    while (ix < nx || iy < ny)
    {
        i32 decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx;
    
        if (decision == 0)
        {
            // Next step is diagonal
            here.x += signX;
            here.y += signY;
    
            ix++;
            iy++;
        }
        else if (decision < 0)
        {
            // Next step is horizontal
            here.x += signX;
            ix++;
        }
        else // if (decision > 0)
        {
            // Next step is vertical
            here.y += signY;
            iy++;
        }
    
        if (IsWalkableNodeAt(here.x, here.y) == false)
            return false;
    }

    return true;
}

std::vector<Vec2D<i32>> StaticMap::MakeOptimizedPathPoints(const std::vector<Vec2D<i32>>& pathPoints, PathOptimizationOption pathOptimizationOption)
{
    std::vector<Vec2D<i32>> ret;

    if (pathPoints.size() == 0)
        return ret;

    // ��� ����ȭ ����
    ret.push_back(pathPoints[0]);

    i32 lastIdx = 0;

    switch (pathOptimizationOption)
    {
        case PathOptimizationOption::BresenhamLines:
        {
            for (i32 idx : std::views::iota(1, std::ssize(pathPoints)))
            {
                if (HasLineOfSight_BresenhamLines(pathPoints[lastIdx], pathPoints[idx]) == false)
                {
                    // �� ���������� �������� ���� ��带 �ִ´�.
                    ret.push_back(pathPoints[idx - 1]);

                    lastIdx = idx - 1;
                }
            }

            break;
        }

        case PathOptimizationOption::OrthogonalSteps:
        {
            for (i32 idx : std::views::iota(1, std::ssize(pathPoints)))
            {
                if (HasLineOfSight_OrthogonalSteps(pathPoints[lastIdx], pathPoints[idx]) == false)
                {
                    // �� ���������� �������� ���� ��带 �ִ´�.
                    ret.push_back(pathPoints[idx - 1]);

                    lastIdx = idx - 1;
                }
            }

            break;
        }

        case PathOptimizationOption::SupercoverLines:
        {
            for (i32 idx : std::views::iota(1, std::ssize(pathPoints)))
            {
                if (HasLineOfSight_SupercoverLines(pathPoints[lastIdx], pathPoints[idx]) == false)
                {
                    // �� ���������� �������� ���� ��带 �ִ´�.
                    ret.push_back(pathPoints[idx - 1]);

                    lastIdx = idx - 1;
                }
            }

            break;
        }

        default:
        {
            ret.clear();

            return ret;
        }
    }

    // ������ ���� ����
    ret.push_back(pathPoints.back());

    return ret;
}

void StaticMap::optimize()
{
    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         this->optimizeNode(x, y);
    //     }
    // }

    std::ranges::for_each(std::views::iota(0, this->GetSize()), [&](i32 idx) {
        i32 x = idx % this->GetWidth();
        i32 y = idx / this->GetWidth();

        // https://en.wikipedia.org/wiki/Cardinal_direction
        // Ordinal directions
        bool chkEast  = false;
        bool chkWest  = false;
        bool chkSouth = false;
        bool chkNorth = false;

        bool chkSouthEast = false;
        bool chkSouthWest = false;
        bool chkNorthEast = false;
        bool chkNorthWest = false;

        const GridNode* eastNode = nullptr;
        const GridNode* westNode = nullptr;
        const GridNode* southNode = nullptr;
        const GridNode* northNode = nullptr;

        const GridNode* southEastNode = nullptr;
        const GridNode* southWestNode = nullptr;
        const GridNode* northEastNode = nullptr;
        const GridNode* northWestNode = nullptr;

        // Neighbors
        NeighborTable&             neighborTable = _neighborTables[idx];
        std::vector<AdjacentNode>& adjNodeTable  = _adjNodeTables[idx];

        std::ranges::fill(neighborTable, false);

        // �ڱ� �ڽſ� ���� ����� �׷��� �������� ���� ����Ŭ�� ����� ���̱� ������ ���� �ʵ��� �Ѵ�.
        // neighborTable[(i32)AdjacentDirection::None] = false;

        // �̵� ������ ������� Ȯ��
        if (IsWalkableNodeAt(x, y) == false)
            return;

        // Check cardinal directions
        chkEast  = this->IsWalkableNodeAt(x + 1, y);
        chkWest  = this->IsWalkableNodeAt(x - 1, y);
        chkSouth = this->IsWalkableNodeAt(x, y + 1);
        chkNorth = this->IsWalkableNodeAt(x, y - 1);

        // Check diagonal directions
        if (_isAllowedDiagonal == true)
        {
            // �ڳʸ� �հ� �� �� �ִ��� �ƴ����� ����Ѵ�.
            if (_isAllowedCorners == true)
            {
                // ���� ���⿡ ���� �־ �ϳ��� ����Ѵ�.
                chkSouthEast = ((chkSouth || chkEast) && this->IsWalkableNodeAt(x + 1, y + 1));
                chkSouthWest = ((chkSouth || chkWest) && this->IsWalkableNodeAt(x - 1, y + 1));
                chkNorthEast = ((chkNorth || chkEast) && this->IsWalkableNodeAt(x + 1, y - 1));
                chkNorthWest = ((chkNorth || chkWest) && this->IsWalkableNodeAt(x - 1, y - 1));
            }
            else
            {
                // ���� ���⿡ ���� ����� �Ѵ�.
                chkSouthEast = ((chkSouth && chkEast) && this->IsWalkableNodeAt(x + 1, y + 1));
                chkSouthWest = ((chkSouth && chkWest) && this->IsWalkableNodeAt(x - 1, y + 1));
                chkNorthEast = ((chkNorth && chkEast) && this->IsWalkableNodeAt(x + 1, y - 1));
                chkNorthWest = ((chkNorth && chkWest) && this->IsWalkableNodeAt(x - 1, y - 1));
            }
        }

        // ��� ������ �̿� ���
        if (chkEast == true)
        {
            eastNode = &(this->GetNodeAt(x + 1, y));
        }

        if (chkWest == true)
        {
            westNode = &(this->GetNodeAt(x - 1, y));
        }

        if (chkSouth == true)
        {
            southNode = &(this->GetNodeAt(x, y + 1));
        }
        if (chkNorth == true)
        {
            northNode = &(this->GetNodeAt(x, y - 1));
        }

        if (chkSouthEast == true)
        {
            southEastNode = &(this->GetNodeAt(x + 1, y + 1));
        }

        if (chkSouthWest == true)
        {
            southWestNode = &(this->GetNodeAt(x - 1, y + 1));
        }

        if (chkNorthEast == true)
        {
            northEastNode = &(this->GetNodeAt(x + 1, y - 1));
        }

        if (chkNorthWest == true)
        {
            northWestNode = &(this->GetNodeAt(x - 1, y - 1));
        }
        
        // ���� ���� ����
        neighborTable[(i32)AdjacentDirection::North] = chkNorth;
        neighborTable[(i32)AdjacentDirection::East]  = chkEast;
        neighborTable[(i32)AdjacentDirection::South] = chkSouth;
        neighborTable[(i32)AdjacentDirection::West]  = chkWest;

        neighborTable[(i32)AdjacentDirection::NorthEast] = chkNorthEast;
        neighborTable[(i32)AdjacentDirection::SouthEast] = chkSouthEast;
        neighborTable[(i32)AdjacentDirection::SouthWest] = chkSouthWest;
        neighborTable[(i32)AdjacentDirection::NorthWest] = chkNorthWest;

        // �Ϲ����� ���� Ž�� ����
        // : �� -> �ϵ� -> �� -> ���� -> �� -> ���� -> �� -> �ϼ�
        //
        // ���⼱ ����� ���� �⺻ �������� Ž���� ���� �밢�� ������ Ž���ϵ��� �Ѵ�.
        // : �� -> �� -> �� -> �� -> �ϼ� -> �ϵ� -> ���� -> ����
        if (neighborTable[(i32)AdjacentDirection::North] == true)
        {
            adjNodeTable.push_back({ northNode, AdjacentDirection::North });
        }

        if (neighborTable[(i32)AdjacentDirection::East] == true)
        {
            adjNodeTable.push_back({ eastNode, AdjacentDirection::East });
        }

        if (neighborTable[(i32)AdjacentDirection::South] == true)
        {
            adjNodeTable.push_back({ southNode, AdjacentDirection::South });
        }

        if (neighborTable[(i32)AdjacentDirection::West] == true)
        {
            adjNodeTable.push_back({ westNode, AdjacentDirection::West });
        }

        if (neighborTable[(i32)AdjacentDirection::NorthWest] == true)
        {
            adjNodeTable.push_back({ northWestNode, AdjacentDirection::NorthWest });
        }

        if (neighborTable[(i32)AdjacentDirection::NorthEast] == true)
        {
            adjNodeTable.push_back({ northEastNode, AdjacentDirection::NorthEast });
        }

        if (neighborTable[(i32)AdjacentDirection::SouthEast] == true)
        {
            adjNodeTable.push_back({ southEastNode, AdjacentDirection::SouthEast });
        }

        if (neighborTable[(i32)AdjacentDirection::SouthWest] == true)
        {
            adjNodeTable.push_back({ southWestNode, AdjacentDirection::SouthWest });
        }
    });
}

void StaticMap::prepareJumpLookUpTable()
{
    // ValidDirLookUpTable
    //     Traveling South: West, Southwest, South, Southeast, East
    //     Traveling Southeast: South, Southeast, East
    //     Traveling East: South, Southeast, East, Northeast, North
    //     Traveling Northeast: East, Northeast, North
    //     Traveling North: East, Northeast, North, Northwest, West
    //     Traveling Northwest: North, Northwest, West
    //     Traveling West: North, Northwest, West, Southwest, South
    //     Traveling Southwest: West, Southwest, South

    using enum AdjacentDirection;

    // NorthWest
    _jumpDirLookUpTables[(i32)NorthWest] = { West, NorthWest, North };

    // North
    _jumpDirLookUpTables[(i32)North] = { West, NorthWest, North, NorthEast, East };

    // NorthEast
    _jumpDirLookUpTables[(i32)NorthEast] = { North, NorthEast, East };

    // kWest
    _jumpDirLookUpTables[(i32)West] = { South, SouthWest, West, NorthWest, North };

    // None : �� ���� ��� ������ �����ϴµ� ����Ѵ�.
    _jumpDirLookUpTables[(i32)None] = { North, NorthEast, East, SouthEast, South, SouthWest, West, NorthWest };

    // East
    _jumpDirLookUpTables[(i32)East] = { North, NorthEast, East, SouthEast, South };

    // SouthWest   
    _jumpDirLookUpTables[(i32)SouthWest] = { South, SouthWest, West };

    // South
    _jumpDirLookUpTables[(i32)South] = { East, SouthEast, South, SouthWest, West };

    // SouthEast
    _jumpDirLookUpTables[(i32)SouthEast] = { East, SouthEast, South };
}

void StaticMap::preprocessPrimaryJumpPoints()
{
    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    // }

    std::ranges::for_each(std::views::iota(0, this->GetSize()), [&](i32 idx) {
        i32 x = idx % this->GetWidth();
        i32 y = idx / this->GetWidth();

        // East Forced Neighbor
        // 1. ������ ���������� -> ������ ��� �밢�����δ� �̵� ����
        // 2. �Ʒ����� ���������� -> ������ �ϴ� �밢�����δ� �̵� ����
        if ((CanMoveByDeltaPos(x, y, 0, -1) == false && CanMoveByDeltaPos(x, y, 1, -1) == true) ||
            (CanMoveByDeltaPos(x, y, 0,  1) == false && CanMoveByDeltaPos(x, y, 1,  1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x + 1, y);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::East;
        }

        // West Forced Neighbor
        // 1. ������ ���������� -> ���� ��� �밢�����δ� �̵� ����
        // 2. �Ʒ����� ���������� -> ���� �ϴ� �밢�����δ� �̵� ����
        if ((CanMoveByDeltaPos(x, y, 0, -1) == false && CanMoveByDeltaPos(x, y, -1, -1) == true) ||
            (CanMoveByDeltaPos(x, y, 0,  1) == false && CanMoveByDeltaPos(x, y, -1,  1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x - 1, y);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::West;
        }
        
        // North Forced Neighbor
        // 1. ������ ���������� -> ���� ��� �밢�����δ� �̵� ����
        // 2. �������� ���������� -> ������ ��� �밢�����δ� �̵� ����
        if ((CanMoveByDeltaPos(x, y, -1, 0) == false && CanMoveByDeltaPos(x, y, -1, -1) == true) ||
            (CanMoveByDeltaPos(x, y,  1, 0) == false && CanMoveByDeltaPos(x, y,  1, -1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x, y - 1);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::North;
        }

        // South Forced Neighbor
        // 1. ������ ���������� -> ���� �ϴ� �밢�����δ� �̵� ����
        // 2. �������� ���������� -> ������ �ϴ� �밢�����δ� �̵� ����
        if ((CanMoveByDeltaPos(x, y, -1, 0) == false && CanMoveByDeltaPos(x, y, -1, 1) == true) ||
            (CanMoveByDeltaPos(x, y,  1, 0) == false && CanMoveByDeltaPos(x, y,  1, 1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x, y + 1);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::South;
        }
    });
}

void StaticMap::preprocessStraightJumpPoints()
{
    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = _width - 1; x >= 0; x--)
    //     {
    //         ...
    //     }
    // 
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    // }

    // East and West Straight Jump Points
    for (i32 y : std::views::iota(0, _height))
    { 
        // �� �ۿ��� �����Ѵٰ� ����(�� ���� Wall)
        i32  distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
        bool jumpPointFound = false;

        // East Straight Jump Points
        // ������ �� -> ���� ������ Ž���� �����ؼ� Primary Jump Point�� Wall �߰� �������κ��� ������ �Ÿ��� �����Ѵ�.
        for (i32 x : std::views::iota(0, _width) | std::views::reverse)
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = 0;

                continue;
            }

            // 1ĭ ��������.
            distance++;

            if (jumpPointFound == false) // Wall�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                // ����� ������ ������ Wall���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = distance;
            }

            // Primary Jump Point �߰�
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::East)
            {
                distance       = 0; // ���� ��� �� Primary Jump Point���� �Ÿ��� 1�� �ǰ� 0 ����
                jumpPointFound = true;
            }
        }

        // �� �ۿ��� �����Ѵٰ� ����(�� ���� Wall)
        distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
        jumpPointFound = false;

        // West Straight Jump Points
        // ���� �� -> ������ ������ Ž���� �����ؼ� Primary Jump Point�� Wall �߰� �������κ��� ������ �Ÿ��� �����Ѵ�.
        for (i32 x : std::views::iota(0, _width))
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = 0;

                continue;
            }

            // 1ĭ ��������.
            distance++;

            if (jumpPointFound == false) // Wall�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                // ����� ������ ������ Wall���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = distance;
            }

            // Primary Jump Point �߰�
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::West)
            {
                distance       = 0; // ���� ��� �� Primary Jump Point���� �Ÿ��� 1�� �ǰ� 0 ����
                jumpPointFound = true;
            }
        }
    }

    // Legacy Code
    // for (i32 x = 0; x < _width; x++)
    // {
    //     for (i32 y = 0; y < _height; y++)
    //     {
    //         ...
    //     }
    // 
    //     for (i32 y = _height - 1; y >= 0; y--)
    //     {
    //         ...
    //     }
    // }

    // North and South Straight Jump Points
    for (i32 x : std::views::iota(0, _width))
    {
        // �� �ۿ��� �����Ѵٰ� ����(�� ���� Wall)
        i32  distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
        bool jumpPointFound = false;

        // North Straight Jump Points
        // ���� �� -> �Ʒ��� ������ Ž���� �����ؼ� Primary Jump Point�� Wall �߰� �������κ��� ������ �Ÿ��� �����Ѵ�.
        for (i32 y : std::views::iota(0, _height))
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = 0;

                continue;
            }

            // 1ĭ ��������.
            distance++;

            if (jumpPointFound == false) // Wall�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                // ����� ������ ������ Wall���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = distance;
            }

            // Primary Jump Point �߰�
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::North)
            {
                distance       = 0; // ���� ��� �� Primary Jump Point���� �Ÿ��� 1�� �ǰ� 0 ����
                jumpPointFound = true;
            }
        }

        // �� �ۿ��� �����Ѵٰ� ����(�� ���� Wall)
        distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
        jumpPointFound = false;

        // South Straight Jump Points
        // �Ʒ��� �� -> ���� ������ Ž���� �����ؼ� Primary Jump Point�� Wall �߰� �������κ��� ������ �Ÿ��� �����Ѵ�.
        for (i32 y : std::views::iota(0, _height) | std::views::reverse)
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall �߰�
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall�κ��� ������ �Ÿ��� 0���� ó���ϱ� ���� -1 ����
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = 0;

                continue;
            }

            // 1ĭ ��������.
            distance++;

            if (jumpPointFound == false) // Wall�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                // ����� ������ ������ Wall���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point�κ��� ������ �Ÿ��� ����ϴ� ��
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = distance;
            }

            // Primary Jump Point �߰�
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::South)
            {
                distance       = 0; // ���� ��� �� Primary Jump Point���� �Ÿ��� 1�� �ǰ� 0 ����
                jumpPointFound = true;
            }
        }
    }
}

void StaticMap::preprocessDiagonalJumpPoints()
{
    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    //     
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    // }
    
    // NorthEast and NorthWest Diagonal Jump Points
    for (i32 y : std::views::iota(0, _height))
    {
        // NorthEast Diagonal Jump Points
        // ����� Row���� Ž���Ͽ� ���� �밢 ������ ���� �밢 ��ġ�� �ݿ��ϸ鼭 ��������.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;
            
            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x + 1, y - 1); // [��]

            // ���� ��ġ�� ���� ���ڶ��� ������ �̴� ���� �밢 ��ġ�� �� ���� ����ٴ� ���� �ǹ��Ѵ�.
            // ��, ���� �밢 ������ ��ȸ�ϴ� ���� �Ұ����� �����̴�.
            if (y == 0 || x == _width - 1)
            {
                // �� ���� ������ �ѷ��ο� �ִٰ� ������ �Ÿ�
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = 0;

                continue;
            }
            
            /**
             * 1) [ ][ ]  2) [X][ ]  3) [ ][X]  4) [ ][ ]
             *    [O][ ]     [O][ ]     [O][ ]     [O][X]
             * 
             * 5) [X][X]  6) [ ][X]  7) [X][ ]  8) [X][X]
             *    [O][ ]     [O][X]     [O][X]     [O][X]
             * 
             * O�� ���� ��ġ��� �������� �� ���� �� �ִ� ��� ����� ���� �ð�ȭ�ϸ� �� ����� ���� ���´�.
             * �Ʒ� if ���� ������� �ʴ� �� ���� ���� 1�� �ۿ� ����(�밢 ��Ұ� ����Ǿ� �־�� ��).
             */
            // ���� �밢 ��ġ���� ���� �밢 ��ġ�� �� �� �ִ��� Ȯ��(�ڳ� ��� X)
            // ���� �ϳ��� ��ġ�� �밢 �������� ������ �� ����(Don't Allow Corners).
            if (IsWalkableNodeAt(x + 1, y) == false ||   // From East [��]
                IsWalkableNodeAt(x, y - 1) == false ||   // From North [��]
                IsWalkableNodeAt(x + 1, y - 1) == false) // From NorthEast [��]
            {
                // ������ �ǻ� �ڵ忡�� "Wall one away"��� �ּ��� �����ִµ�
                // �̰� ������ �� ĭ ��������� ������ �� ĭ�� ���� ���� �ִٴ� ������ ǥ���� �ణ �������̶� �ָ��ϴ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = 0;

                continue;
            }

            // --------------------------------------------------
            // ������� ������ ���� �밢 ��ġ�� �շ������� �̵� ���⿡ �ڳʰ� ���ٴ� ���� �����Ѵ�.
            // --------------------------------------------------

            /**
             * �밢�� ������ Straight Jump Point�� ����� �� �ִ�(������ ���� ȯ�� ���� ��� ����).
             * 
             * ������ �ǻ� �ڵ带 ���� �̷� ������ �ڵ尡 �ۼ��Ǿ� �ִ�.
             * : if (IsWalkableAt(x + 1, y) == true && IsWalkableAt(x, y - 1) == true)
             * 
             * ������ �밢�� �������� �շ� �ִٴ� ���� �����ϰ� �ִµ� �밢 ��Ҹ� �����Ͽ� ���� ��ġ�� üũ�ϴ� ������ �𸣰ھ �ش� �ڵ�� �����ߴ�.
             */
            // ���� �밢 ��Ҹ� �������� �� ���� ���� ������ Straight Jump Point�� ���� ���� ������ �ִٸ� ž���Ѵ�.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::North] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::East] > 0)
            {
                // ������ �ǻ� �ڵ忡�� "Straight jump point one away"��� �����ִ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = 1;

                continue;
            }

            // ���� �밢 ��ġ���� ���� ��ġ�� �̵��� ���� ������ ���� �Ÿ��� ȯ�� ����Ʈ�� ã�� �� �� �����̴�.
            // �̷� ���� ���� �밢 ������ �����Ͽ� ����Ѵ�.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast];

            if (prevDistance <= 0)
            {
                // ���� ���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // ���� Diagonal Jump Point�� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = prevDistance + 1;
            }
        }

        // NorthWest Diagonal Jump Points
        // ����� Row���� Ž���Ͽ� ���� �밢 ������ ���� �밢 ��ġ�� �ݿ��ϸ鼭 ��������.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x - 1, y - 1); // [��]

            // ���� ��ġ�� ���� ���ڶ��� ������ �̴� ���� �밢 ��ġ�� �� ���� ����ٴ� ���� �ǹ��Ѵ�.
            // ��, ���� �밢 ������ ��ȸ�ϴ� ���� �Ұ����� �����̴�.
            if (y == 0 || x == 0)
            {
                // �� ���� ������ �ѷ��ο� �ִٰ� ������ �Ÿ�
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 0;

                continue;
            }

            // ���� �밢 ��ġ���� ���� �밢 ��ġ�� �� �� �ִ��� Ȯ��(�ڳ� ��� X)
            // ���� �ϳ��� ��ġ�� �밢 �������� ������ �� ����(Don't Allow Corners).
            if (IsWalkableNodeAt(x - 1, y) == false ||   // From West [��]
                IsWalkableNodeAt(x, y - 1) == false ||   // From North [��]
                IsWalkableNodeAt(x - 1, y - 1) == false) // From NorthWest [��]
            {
                // ������ �ǻ� �ڵ忡�� "Wall one away"��� �ּ��� �����ִµ�
                // �̰� ������ �� ĭ ��������� ������ �� ĭ�� ���� ���� �ִٴ� ������ ǥ���� �ణ �������̶� �ָ��ϴ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 0;

                continue;
            }

            // --------------------------------------------------
            // ������� ������ ���� �밢 ��ġ�� �շ������� �̵� ���⿡ �ڳʰ� ���ٴ� ���� �����Ѵ�.
            // --------------------------------------------------

            // ���� �밢 ��Ҹ� �������� �� ���� ���� ������ Straight Jump Point�� ���� ���� ������ �ִٸ� ž���Ѵ�.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::North] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::West] > 0)
            {
                // ������ �ǻ� �ڵ忡�� "Straight jump point one away"��� �����ִ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 1;

                continue;
            }

            // ���� �밢 ��ġ���� ���� ��ġ�� �̵��� ���� ������ ���� �Ÿ��� ȯ�� ����Ʈ�� ã�� �� �� �����̴�.
            // �̷� ���� ���� �밢 ������ �����Ͽ� ����Ѵ�.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest];

            if (prevDistance <= 0)
            {
                // ���� ���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // ���� Diagonal Jump Point�� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = prevDistance + 1;
            }
        }
    }

    // Legacy Code
    // for (i32 y = _height - 1; y >= 0; y--)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    //     
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         ...
    //     }
    // }
    
    // SouthEast and SouthWest Diagonal Jump Points
    for (i32 y : std::views::iota(0, _height) | std::views::reverse)
    {
        // SouthEast Diagonal Jump Points
        // �ϴ��� Row���� Ž���Ͽ� ���� �밢 ������ ���� �밢 ��ġ�� �ݿ��ϸ鼭 �ö󰣴�.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x + 1, y + 1); // [��]

            // ���� ��ġ�� ���� ���ڶ��� ������ �̴� ���� �밢 ��ġ�� �� ���� ����ٴ� ���� �ǹ��Ѵ�.
            // ��, ���� �밢 ������ ��ȸ�ϴ� ���� �Ұ����� �����̴�.
            if (y == _height - 1 || x == _width - 1)
            {
                // �� ���� ������ �ѷ��ο� �ִٰ� ������ �Ÿ�
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 0;

                continue;
            }

            // ���� �밢 ��ġ���� ���� �밢 ��ġ�� �� �� �ִ��� Ȯ��(�ڳ� ��� X)
            // ���� �ϳ��� ��ġ�� �밢 �������� ������ �� ����(Don't Allow Corners).
            if (IsWalkableNodeAt(x + 1, y) == false ||   // From East [��]
                IsWalkableNodeAt(x, y + 1) == false ||   // From South [��]
                IsWalkableNodeAt(x + 1, y + 1) == false) // From SouthEast [��]
            {
                // ������ �ǻ� �ڵ忡�� "Wall one away"��� �ּ��� �����ִµ�
                // �̰� ������ �� ĭ ��������� ������ �� ĭ�� ���� ���� �ִٴ� ������ ǥ���� �ణ �������̶� �ָ��ϴ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 0;

                continue;
            }

            // --------------------------------------------------
            // ������� ������ ���� �밢 ��ġ�� �շ������� �̵� ���⿡ �ڳʰ� ���ٴ� ���� �����Ѵ�.
            // --------------------------------------------------

            // ���� �밢 ��Ҹ� �������� �� ���� ���� ������ Straight Jump Point�� ���� ���� ������ �ִٸ� ž���Ѵ�.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::South] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::East] > 0)
            {
                // ������ �ǻ� �ڵ忡�� "Straight jump point one away"��� �����ִ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 1;

                continue;
            }

            // ���� �밢 ��ġ���� ���� ��ġ�� �̵��� ���� ������ ȯ�� ����Ʈ�� ã�� �� �� ������.
            // �̷� ���� ���� �밢 ��Ҹ� ����Ѵ�.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast];
            
            if (prevDistance <= 0)
            {
                // ���� ���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // ���� Diagonal Jump Point�� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = prevDistance + 1;
            }
        }
        
        // SouthWest Diagonal Jump Points
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x - 1, y + 1); // [��]

            // ���� ��ġ�� ���� ���ڶ��� ������ �̴� ���� �밢 ��ġ�� �� ���� ����ٴ� ���� �ǹ��Ѵ�.
            // ��, ���� �밢 ������ ��ȸ�ϴ� ���� �Ұ����� �����̴�.
            if (y == _height - 1 || 0 == x)
            {
                // �� ���� ������ �ѷ��ο� �ִٰ� ������ �Ÿ�
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 0;

                continue;
            }

            // ���� �밢 ��ġ���� ���� �밢 ��ġ�� �� �� �ִ��� Ȯ��(�ڳ� ��� X)
            // ���� �ϳ��� ��ġ�� �밢 �������� ������ �� ����(Don't Allow Corners).
            if (IsWalkableNodeAt(x - 1, y) == false ||   // From West [��]
                IsWalkableNodeAt(x, y + 1) == false ||   // From South [��]
                IsWalkableNodeAt(x - 1, y + 1) == false) // From SouthWest [��]
            {
                // ������ �ǻ� �ڵ忡�� "Wall one away"��� �ּ��� �����ִµ�
                // �̰� ������ �� ĭ ��������� ������ �� ĭ�� ���� ���� �ִٴ� ������ ǥ���� �ణ �������̶� �ָ��ϴ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 0;

                continue;
            }

            // --------------------------------------------------
            // ������� ������ ���� �밢 ��ġ�� �շ������� �̵� ���⿡ �ڳʰ� ���ٴ� ���� �����Ѵ�.
            // --------------------------------------------------

            // ���� �밢 ��ġ�� Straight Jump Point�� ������ �ִٸ� ž���Ѵ�.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::South] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::West] > 0)
            {
                // ������ �ǻ� �ڵ忡�� "Straight jump point one away"��� �����ִ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 1;

                continue;
            }

            // ���� �밢 ��ġ���� ���� ��ġ�� �̵��� ���� ������ ȯ�� ����Ʈ�� ã�� �� �� ������.
            // �̷� ���� ���� �밢 ��Ҹ� ����Ѵ�.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest];

            if (prevDistance <= 0)
            {
                // ���� ���� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // ���� Diagonal Jump Point�� �Ÿ��� ����Ѵ�.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = prevDistance + 1;
            }
        }
    }
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
