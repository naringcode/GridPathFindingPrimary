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

    // 노드 정보
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

    // 최적화 진행
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

    // 노드 정보
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

    // 최적화 진행
    instance->optimize();

    if (this->IsOptimizedForJpsPlus() == true)
    {
        instance->OptimizeForJpsPlus();
    }

    return std::shared_ptr<StaticMap>{ instance, Deleter{ } };
}

bool StaticMap::OptimizeForJpsPlus()
{
    // 코너 자체는 사용하지 않지만 JPS+ 맵을 계산하는 과정에서 코너 여부를 조회한다(_isAllowedCorners가 true여야 함).
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
    
    // 기울기가 완만하면(1보다 작으면) x++로 p값을 체크해서 y를 증가시켜야 한다.
    if (dy < dx)
    {
        // x축이 처리의 기준이기에 x값이 작은 점을 기준으로 삼는다.
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
    
        // dy가 음수였던 상황(abs()를 적용시키면 dy는 x축 대칭이 되기 때문에 increment도 대칭시켜야 한다.
        if (endY - y < 0)
        {
            increment = -1;
        }
    
        errorBound = dyDouble - dx; // p0 = 2dy - dx
    
        while (x <= endX)
        {
            // Wall 발견
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
    
            // 기존 공식에선 이렇게 사용함
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
    
    // 기울기가 가파르면(1보다 크면) y++로 p값을 체크해서 x를 증가시켜야 한다.
    // y = x와 대칭이기 때문에 x와 y만 서로 치환하면 된다.
    else
    {
        // y축이 처리의 기준이기에 y값이 작은 점을 기준으로 삼는다.
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
    
        // dx가 음수였던 상황(abs()를 적용시키면 dx는 y축 대칭이 되기 때문에 increment도 대칭시켜야 한다.
        if (endX - x < 0)
        {
            increment = -1;
        }
    
        errorBound = dxDouble - dy; // p0 = 2dx - dy
    
        while (y <= endY)
        {
            // Wall 발견
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
    
            // 기존 공식에선 이렇게 사용함
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
     * 테스트해보니 Bresenham Line은 어떤 타일을 통과하는지 감지하지 못 하는 문제가 발생했다.
     *
     * https://www.redblobgames.com/grids/line-drawing/
     * 
     * 대안책
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

        // nx와 ny가 실수이고 이 값을 0으로 나누면 inf가 나온다(왼쪽 피연산자가 양수이기 때문에 양의 무한대로 나옴).
        i32 chkNx = (nx != 0) ? (nx * 10) : 1;
        i32 chkNy = (ny != 0) ? (ny * 10) : 1;

        // 양변에 10을 곱한 다음 nx와 ny를 추가적으로 곱한 식
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
     * 테스트해보니 Bresenham Line은 어떤 타일을 통과하는지 감지하지 못 하는 문제가 발생했다.
     *
     * https://www.redblobgames.com/grids/line-drawing/
     * 
     * 대안책
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

    // 경로 최적화 시작
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
                    // 현 시점에서는 막혔으니 이전 노드를 넣는다.
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
                    // 현 시점에서는 막혔으니 이전 노드를 넣는다.
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
                    // 현 시점에서는 막혔으니 이전 노드를 넣는다.
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

    // 마지막 지점 저장
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

        // 자기 자신에 대한 등록은 그래프 차원에서 보면 사이클이 생기는 것이기 때문에 하지 않도록 한다.
        // neighborTable[(i32)AdjacentDirection::None] = false;

        // 이동 가능한 노드인지 확인
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
            // 코너를 뚫고 갈 수 있는지 아닌지를 고려한다.
            if (_isAllowedCorners == true)
            {
                // 진행 방향에 벽이 있어도 하나는 허용한다.
                chkSouthEast = ((chkSouth || chkEast) && this->IsWalkableNodeAt(x + 1, y + 1));
                chkSouthWest = ((chkSouth || chkWest) && this->IsWalkableNodeAt(x - 1, y + 1));
                chkNorthEast = ((chkNorth || chkEast) && this->IsWalkableNodeAt(x + 1, y - 1));
                chkNorthWest = ((chkNorth || chkWest) && this->IsWalkableNodeAt(x - 1, y - 1));
            }
            else
            {
                // 진행 방향에 벽이 없어야 한다.
                chkSouthEast = ((chkSouth && chkEast) && this->IsWalkableNodeAt(x + 1, y + 1));
                chkSouthWest = ((chkSouth && chkWest) && this->IsWalkableNodeAt(x - 1, y + 1));
                chkNorthEast = ((chkNorth && chkEast) && this->IsWalkableNodeAt(x + 1, y - 1));
                chkNorthWest = ((chkNorth && chkWest) && this->IsWalkableNodeAt(x - 1, y - 1));
            }
        }

        // 사용 가능한 이웃 노드
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
        
        // 인접 정보 설정
        neighborTable[(i32)AdjacentDirection::North] = chkNorth;
        neighborTable[(i32)AdjacentDirection::East]  = chkEast;
        neighborTable[(i32)AdjacentDirection::South] = chkSouth;
        neighborTable[(i32)AdjacentDirection::West]  = chkWest;

        neighborTable[(i32)AdjacentDirection::NorthEast] = chkNorthEast;
        neighborTable[(i32)AdjacentDirection::SouthEast] = chkSouthEast;
        neighborTable[(i32)AdjacentDirection::SouthWest] = chkSouthWest;
        neighborTable[(i32)AdjacentDirection::NorthWest] = chkNorthWest;

        // 일반적인 방위 탐색 순서
        // : 북 -> 북동 -> 동 -> 남동 -> 남 -> 남서 -> 서 -> 북서
        //
        // 여기선 비용이 적은 기본 방위부터 탐색한 다음 대각선 방위를 탐색하도록 한다.
        // : 북 -> 동 -> 남 -> 서 -> 북서 -> 북동 -> 남동 -> 남서
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

    // None : 이 값은 모든 방향을 지정하는데 사용한다.
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
        // 1. 위쪽이 막혀있지만 -> 오른쪽 상단 대각선으로는 이동 가능
        // 2. 아래쪽이 막혀있지만 -> 오른쪽 하단 대각선으로는 이동 가능
        if ((CanMoveByDeltaPos(x, y, 0, -1) == false && CanMoveByDeltaPos(x, y, 1, -1) == true) ||
            (CanMoveByDeltaPos(x, y, 0,  1) == false && CanMoveByDeltaPos(x, y, 1,  1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x + 1, y);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::East;
        }

        // West Forced Neighbor
        // 1. 위쪽이 막혀있지만 -> 왼쪽 상단 대각선으로는 이동 가능
        // 2. 아래쪽이 막혀있지만 -> 왼쪽 하단 대각선으로는 이동 가능
        if ((CanMoveByDeltaPos(x, y, 0, -1) == false && CanMoveByDeltaPos(x, y, -1, -1) == true) ||
            (CanMoveByDeltaPos(x, y, 0,  1) == false && CanMoveByDeltaPos(x, y, -1,  1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x - 1, y);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::West;
        }
        
        // North Forced Neighbor
        // 1. 왼쪽이 막혀있지만 -> 왼쪽 상단 대각선으로는 이동 가능
        // 2. 오른쪽이 막혀있지만 -> 오른쪽 상단 대각선으로는 이동 가능
        if ((CanMoveByDeltaPos(x, y, -1, 0) == false && CanMoveByDeltaPos(x, y, -1, -1) == true) ||
            (CanMoveByDeltaPos(x, y,  1, 0) == false && CanMoveByDeltaPos(x, y,  1, -1) == true))
        {
            i32 idx = this->ConvertToNodeIdx(x, y - 1);

            _jumpPointNodes[idx].primaryIndegreeFlags |= (i32)JpsDirectionFlags::North;
        }

        // South Forced Neighbor
        // 1. 왼쪽이 막혀있지만 -> 왼쪽 하단 대각선으로는 이동 가능
        // 2. 오른쪽이 막혀있지만 -> 오른쪽 하단 대각선으로는 이동 가능
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
        // 맵 밖에서 시작한다고 가정(맵 밖은 Wall)
        i32  distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
        bool jumpPointFound = false;

        // East Straight Jump Points
        // 오른쪽 끝 -> 왼쪽 끝으로 탐색을 진행해서 Primary Jump Point나 Wall 발견 지점으로부터 떨어진 거리를 기입한다.
        for (i32 x : std::views::iota(0, _width) | std::views::reverse)
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall 발견
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = 0;

                continue;
            }

            // 1칸 떨어진다.
            distance++;

            if (jumpPointFound == false) // Wall로부터 떨어진 거리를 계산하는 중
            {
                // 양수를 뒤집은 음수를 Wall과의 거리로 사용한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point로부터 떨어진 거리를 계산하는 중
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::East] = distance;
            }

            // Primary Jump Point 발견
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::East)
            {
                distance       = 0; // 다음 계산 시 Primary Jump Point와의 거리가 1이 되게 0 대입
                jumpPointFound = true;
            }
        }

        // 맵 밖에서 시작한다고 가정(맵 밖은 Wall)
        distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
        jumpPointFound = false;

        // West Straight Jump Points
        // 왼쪽 끝 -> 오른쪽 끝으로 탐색을 진행해서 Primary Jump Point나 Wall 발견 지점으로부터 떨어진 거리를 기입한다.
        for (i32 x : std::views::iota(0, _width))
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall 발견
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = 0;

                continue;
            }

            // 1칸 떨어진다.
            distance++;

            if (jumpPointFound == false) // Wall로부터 떨어진 거리를 계산하는 중
            {
                // 양수를 뒤집은 음수를 Wall과의 거리로 사용한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point로부터 떨어진 거리를 계산하는 중
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::West] = distance;
            }

            // Primary Jump Point 발견
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::West)
            {
                distance       = 0; // 다음 계산 시 Primary Jump Point와의 거리가 1이 되게 0 대입
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
        // 맵 밖에서 시작한다고 가정(맵 밖은 Wall)
        i32  distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
        bool jumpPointFound = false;

        // North Straight Jump Points
        // 위쪽 끝 -> 아래쪽 끝으로 탐색을 진행해서 Primary Jump Point나 Wall 발견 지점으로부터 떨어진 거리를 기입한다.
        for (i32 y : std::views::iota(0, _height))
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall 발견
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = 0;

                continue;
            }

            // 1칸 떨어진다.
            distance++;

            if (jumpPointFound == false) // Wall로부터 떨어진 거리를 계산하는 중
            {
                // 양수를 뒤집은 음수를 Wall과의 거리로 사용한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point로부터 떨어진 거리를 계산하는 중
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::North] = distance;
            }

            // Primary Jump Point 발견
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::North)
            {
                distance       = 0; // 다음 계산 시 Primary Jump Point와의 거리가 1이 되게 0 대입
                jumpPointFound = true;
            }
        }

        // 맵 밖에서 시작한다고 가정(맵 밖은 Wall)
        distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
        jumpPointFound = false;

        // South Straight Jump Points
        // 아래쪽 끝 -> 위쪽 끝으로 탐색을 진행해서 Primary Jump Point나 Wall 발견 지점으로부터 떨어진 거리를 기입한다.
        for (i32 y : std::views::iota(0, _height) | std::views::reverse)
        {
            i32 idx = this->ConvertToNodeIdx(x, y);

            // Wall 발견
            if (IsWalkableNodeAt(x, y) == false)
            {
                distance       = -1; // Wall로부터 떨어진 거리를 0으로 처리하기 위해 -1 대입
                jumpPointFound = false;

                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = 0;

                continue;
            }

            // 1칸 떨어진다.
            distance++;

            if (jumpPointFound == false) // Wall로부터 떨어진 거리를 계산하는 중
            {
                // 양수를 뒤집은 음수를 Wall과의 거리로 사용한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = -distance;
            }
            else // if (jumpPointFound == true) // Jump Point로부터 떨어진 거리를 계산하는 중
            {
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::South] = distance;
            }

            // Primary Jump Point 발견
            if (_jumpPointNodes[idx].primaryIndegreeFlags & (i32)JpsDirectionFlags::South)
            {
                distance       = 0; // 다음 계산 시 Primary Jump Point와의 거리가 1이 되게 0 대입
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
        // 상단의 Row부터 탐색하여 이전 대각 정보를 현재 대각 위치에 반영하면서 내려간다.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;
            
            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x + 1, y - 1); // [↗]

            // 현재 위치가 맵의 끝자락에 있으며 이는 이전 대각 위치가 맵 밖을 벗어난다는 것을 의미한다.
            // 즉, 이전 대각 정보를 조회하는 것이 불가능한 상태이다.
            if (y == 0 || x == _width - 1)
            {
                // 맵 밖은 벽으로 둘러싸여 있다고 가정한 거리
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
             * O를 현재 위치라고 가정했을 때 나올 수 있는 모든 경우의 수를 시각화하면 위 경우의 수가 나온다.
             * 아래 if 문을 통과하지 않는 건 벽이 없는 1번 밖에 없다(대각 요소가 개방되어 있어야 함).
             */
            // 이전 대각 위치에서 현재 대각 위치로 올 수 있는지 확인(코너 허용 X)
            // 벽을 하나라도 걸치면 대각 방향으로 움직일 수 없다(Don't Allow Corners).
            if (IsWalkableNodeAt(x + 1, y) == false ||   // From East [←]
                IsWalkableNodeAt(x, y - 1) == false ||   // From North [↓]
                IsWalkableNodeAt(x + 1, y - 1) == false) // From NorthEast [↙]
            {
                // 원문의 의사 코드에는 "Wall one away"라는 주석만 나와있는데
                // 이게 벽에서 한 칸 떨어지라는 뜻인지 한 칸을 가면 벽이 있다는 뜻인지 표현이 약간 중의적이라 애매하다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = 0;

                continue;
            }

            // --------------------------------------------------
            // 여기까지 왔으면 이전 대각 위치가 뚫려있으며 이동 방향에 코너가 없다는 것을 보장한다.
            // --------------------------------------------------

            /**
             * 대각선 방향의 Straight Jump Point와 연결될 수 있다(점프를 통한 환승 지점 계산 가능).
             * 
             * 원서의 의사 코드를 보면 이런 느낌의 코드가 작성되어 있다.
             * : if (IsWalkableAt(x + 1, y) == true && IsWalkableAt(x, y - 1) == true)
             * 
             * 하지만 대각선 방향으로 뚫려 있다는 것을 보장하고 있는데 대각 요소를 분해하여 이전 위치를 체크하는 이유를 모르겠어서 해당 코드는 제거했다.
             */
            // 이전 대각 요소를 분해했을 때 직선 점프 정보인 Straight Jump Point에 대한 값을 가지고 있다면 탑승한다.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::North] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::East] > 0)
            {
                // 원문의 의사 코드에는 "Straight jump point one away"라고 나와있다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = 1;

                continue;
            }

            // 이전 대각 위치에서 현재 위치로 이동할 수는 있지만 직선 거리의 환승 포인트는 찾지 못 한 상태이다.
            // 이럴 때는 이전 대각 정보를 누적하여 계승한다.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast];

            if (prevDistance <= 0)
            {
                // 이전 벽의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // 이전 Diagonal Jump Point의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthEast] = prevDistance + 1;
            }
        }

        // NorthWest Diagonal Jump Points
        // 상단의 Row부터 탐색하여 이전 대각 정보를 현재 대각 위치에 반영하면서 내려간다.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x - 1, y - 1); // [↖]

            // 현재 위치가 맵의 끝자락에 있으며 이는 이전 대각 위치가 맵 밖을 벗어난다는 것을 의미한다.
            // 즉, 이전 대각 정보를 조회하는 것이 불가능한 상태이다.
            if (y == 0 || x == 0)
            {
                // 맵 밖은 벽으로 둘러싸여 있다고 가정한 거리
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 0;

                continue;
            }

            // 이전 대각 위치에서 현재 대각 위치로 올 수 있는지 확인(코너 허용 X)
            // 벽을 하나라도 걸치면 대각 방향으로 움직일 수 없다(Don't Allow Corners).
            if (IsWalkableNodeAt(x - 1, y) == false ||   // From West [→]
                IsWalkableNodeAt(x, y - 1) == false ||   // From North [↓]
                IsWalkableNodeAt(x - 1, y - 1) == false) // From NorthWest [↘]
            {
                // 원문의 의사 코드에는 "Wall one away"라는 주석만 나와있는데
                // 이게 벽에서 한 칸 떨어지라는 뜻인지 한 칸을 가면 벽이 있다는 뜻인지 표현이 약간 중의적이라 애매하다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 0;

                continue;
            }

            // --------------------------------------------------
            // 여기까지 왔으면 이전 대각 위치가 뚫려있으며 이동 방향에 코너가 없다는 것을 보장한다.
            // --------------------------------------------------

            // 이전 대각 요소를 분해했을 때 직선 점프 정보인 Straight Jump Point에 대한 값을 가지고 있다면 탑승한다.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::North] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::West] > 0)
            {
                // 원문의 의사 코드에는 "Straight jump point one away"라고 나와있다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = 1;

                continue;
            }

            // 이전 대각 위치에서 현재 위치로 이동할 수는 있지만 직선 거리의 환승 포인트는 찾지 못 한 상태이다.
            // 이럴 때는 이전 대각 정보를 누적하여 계승한다.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest];

            if (prevDistance <= 0)
            {
                // 이전 벽의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::NorthWest] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // 이전 Diagonal Jump Point의 거리를 계승한다.
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
        // 하단의 Row부터 탐색하여 이전 대각 정보를 현재 대각 위치에 반영하면서 올라간다.
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x + 1, y + 1); // [↘]

            // 현재 위치가 맵의 끝자락에 있으며 이는 이전 대각 위치가 맵 밖을 벗어난다는 것을 의미한다.
            // 즉, 이전 대각 정보를 조회하는 것이 불가능한 상태이다.
            if (y == _height - 1 || x == _width - 1)
            {
                // 맵 밖은 벽으로 둘러싸여 있다고 가정한 거리
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 0;

                continue;
            }

            // 이전 대각 위치에서 현재 대각 위치로 올 수 있는지 확인(코너 허용 X)
            // 벽을 하나라도 걸치면 대각 방향으로 움직일 수 없다(Don't Allow Corners).
            if (IsWalkableNodeAt(x + 1, y) == false ||   // From East [←]
                IsWalkableNodeAt(x, y + 1) == false ||   // From South [↑]
                IsWalkableNodeAt(x + 1, y + 1) == false) // From SouthEast [↖]
            {
                // 원문의 의사 코드에는 "Wall one away"라는 주석만 나와있는데
                // 이게 벽에서 한 칸 떨어지라는 뜻인지 한 칸을 가면 벽이 있다는 뜻인지 표현이 약간 중의적이라 애매하다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 0;

                continue;
            }

            // --------------------------------------------------
            // 여기까지 왔으면 이전 대각 위치가 뚫려있으며 이동 방향에 코너가 없다는 것을 보장한다.
            // --------------------------------------------------

            // 이전 대각 요소를 분해했을 때 직선 점프 정보인 Straight Jump Point에 대한 값을 가지고 있다면 탑승한다.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::South] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::East] > 0)
            {
                // 원문의 의사 코드에는 "Straight jump point one away"라고 나와있다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = 1;

                continue;
            }

            // 이전 대각 위치에서 현재 위치로 이동할 수는 있지만 환승 포인트는 찾지 못 한 상태임.
            // 이럴 때는 이전 대각 요소를 계승한다.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast];
            
            if (prevDistance <= 0)
            {
                // 이전 벽의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // 이전 Diagonal Jump Point의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthEast] = prevDistance + 1;
            }
        }
        
        // SouthWest Diagonal Jump Points
        for (i32 x : std::views::iota(0, _width))
        {
            if (IsWalkableNodeAt(x, y) == false)
                continue;

            i32 idx     = this->ConvertToNodeIdx(x, y);
            i32 prevIdx = this->ConvertToNodeIdx(x - 1, y + 1); // [↙]

            // 현재 위치가 맵의 끝자락에 있으며 이는 이전 대각 위치가 맵 밖을 벗어난다는 것을 의미한다.
            // 즉, 이전 대각 정보를 조회하는 것이 불가능한 상태이다.
            if (y == _height - 1 || 0 == x)
            {
                // 맵 밖은 벽으로 둘러싸여 있다고 가정한 거리
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 0;

                continue;
            }

            // 이전 대각 위치에서 현재 대각 위치로 올 수 있는지 확인(코너 허용 X)
            // 벽을 하나라도 걸치면 대각 방향으로 움직일 수 없다(Don't Allow Corners).
            if (IsWalkableNodeAt(x - 1, y) == false ||   // From West [→]
                IsWalkableNodeAt(x, y + 1) == false ||   // From South [↑]
                IsWalkableNodeAt(x - 1, y + 1) == false) // From SouthWest [↗]
            {
                // 원문의 의사 코드에는 "Wall one away"라는 주석만 나와있는데
                // 이게 벽에서 한 칸 떨어지라는 뜻인지 한 칸을 가면 벽이 있다는 뜻인지 표현이 약간 중의적이라 애매하다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 0;

                continue;
            }

            // --------------------------------------------------
            // 여기까지 왔으면 이전 대각 위치가 뚫려있으며 이동 방향에 코너가 없다는 것을 보장한다.
            // --------------------------------------------------

            // 이전 대각 위치가 Straight Jump Point를 가지고 있다면 탑승한다.
            if (_jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::South] > 0 ||
                _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::West] > 0)
            {
                // 원문의 의사 코드에는 "Straight jump point one away"라고 나와있다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = 1;

                continue;
            }

            // 이전 대각 위치에서 현재 위치로 이동할 수는 있지만 환승 포인트는 찾지 못 한 상태임.
            // 이럴 때는 이전 대각 요소를 계승한다.
            i32 prevDistance = _jumpPointNodes[prevIdx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest];

            if (prevDistance <= 0)
            {
                // 이전 벽의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = prevDistance - 1;
            }
            else // if (prevDistance > 0)
            {
                // 이전 Diagonal Jump Point의 거리를 계승한다.
                _jumpPointNodes[idx].jumpDistanceTable[(i32)AdjacentDirection::SouthWest] = prevDistance + 1;
            }
        }
    }
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
