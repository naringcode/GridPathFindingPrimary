// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Map:StaticMap;

import :Node;

import PathFinding.Algorithm;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class DynamicMap;

    /**
     * JPS+를 사용하기 위해 정보를 가공하는 방식도 포함하고 있다.
     * 알고리즘 동작 방식은 JPS+ 구현 코드와 원서를 참고하도록 한다.
     * 
     * 원서 기준으로 JPS+는 코너를 허용하지 않기에 이를 허용하려면 강제 이웃을 계산하는 방식을 바꿔야 한다.
     */
    /**
     * flood-fill 알고리즘을 써서 갈 수 없는 영역을 구분하는 방법도 일단은 생각하도록 한다(당장 구현할 것은 아님).
     * 
     * DynamicMap -> StaticMap -> PathFinding
     * 
     * StaticMap은 맵의 정보가 정적인 것을 보장하기 때문에 최적화해서 사용해도 된다.
     * 또한 DynamicMap과 마찬가지로 길찾기에 대한 정보를 제공하긴 해도 실제 길찾기 과정은 수행하지 않는다.
     */
    class StaticMap final
    {
    public:
        // 모든 방향을 조회하여 이동 가능한 인접 노드가 존재하는지 확인하기 위한 테이블
        using NeighborTable = std::array<bool, (i32)AdjacentDirection::NumDirections>;

        // 이동 가능한 노드의 인접 정보를 표현하기 위한 구조체
        struct AdjacentNode
        {
            const GridNode*   toAdjNode;
            AdjacentDirection toDir;
        };

        /*************************
        *      For JPS Plus      *
        *************************/
        struct JumpPointNode
        {
            // Primary Jump Point로 들어올 때의 방향 플래그가 저장된다(JpsDirectionFlags).
            i32 primaryIndegreeFlags = 0;

            // 1. 가장 인접한 방향의 Jump Point와의 거리가 저장된다.
            // 2. 인접 방향으로 Jump Point를 조회할 수 없으면 벽과의 거리가 음수로 저장된다.
            i32 jumpDistanceTable[(i32)AdjacentDirection::NumDirections];
        };

        /*************************************
        *      Path Points Optimization      *
        *************************************/
        enum class PathOptimizationOption
        {
            BresenhamLines,
            OrthogonalSteps,
            SupercoverLines,

            Max
        };

    private:
        struct Deleter
        {
            void operator()(StaticMap* instance) const;
        };

    private:
        /*************************
        *      Rule of Five      *
        *************************/
        explicit StaticMap() = default; // constructor
        ~StaticMap() = default; // destructor

        StaticMap(const StaticMap& rhs) = delete; // copy constructor
        StaticMap& operator=(const StaticMap& rhs) = delete; // copy assignment

        StaticMap(StaticMap&& rhs) noexcept = delete; // move constructor
        StaticMap& operator=(StaticMap&& rhs) noexcept = delete; // move assignment

    public:
        static std::shared_ptr<StaticMap> Create(std::shared_ptr<DynamicMap>& dynamicMap, bool allowDiagonal, bool allowCorners);
        // static std::shared_ptr<StaticMap> Create(std::string_view filename, bool allowDiagonal, bool allowCorners);

    public:
        std::shared_ptr<StaticMap> Clone() const;

    public:
        /*************************
        *      Optimization      *
        *************************/
        bool OptimizeForJpsPlus(); // JPS+를 사용하기 위해선 해당 함수를 사전에 호출해야 함.

    public:
        /**************************
        *      Line-Of-Sight      *
        **************************/
        bool HasLineOfSight_BresenhamLines(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const;
        bool HasLineOfSight_OrthogonalSteps(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const;
        bool HasLineOfSight_SupercoverLines(const Vec2D<i32>& startPos, const Vec2D<i32>& destPos) const;

        std::vector<Vec2D<i32>> MakeOptimizedPathPoints(const std::vector<Vec2D<i32>>& pathPoints, PathOptimizationOption pathOptimizationOption);

    public:
        /*********************
        *      Map Info      *
        *********************/
        bool IsValidPos(i32 x, i32 y) const
        {
            if (x < 0 || x >= _width || y < 0 || y >= _height)
                return false;

            return true;
        }

        i32 ConvertToNodeIdx(i32 x, i32 y) const
        {
            // if (IsValidPos(x, y) == false)
            //     return -1;

            return x + (y * _width);
        }

        i32 GetWidth() const
        {
            return _width;
        }

        i32 GetHeight() const
        {
            return _height;
        }

        i32 GetSize() const
        {
            return _width * _height;
        }

        bool IsAllowedDiagonal() const
        {
            return _isAllowedDiagonal;
        }

        bool IsAllowedCorners() const
        {
            return _isAllowedCorners;
        }

    public:
        /**********************
        *      Node Info      *
        **********************/
        const GridNode& GetNodeAt(i32 x, i32 y) const
        {
            if (IsValidPos(x, y) == false)
                return kNullNodeOutOfBounds;

            return _nodes[ConvertToNodeIdx(x, y)];
        }

        NodeState GetNodeStateAt(i32 x, i32 y) const
        {
            if (IsValidPos(x, y) == false)
                return NodeState::Invalid;

            return _nodes[ConvertToNodeIdx(x, y)].state;
        }

        bool IsWalkableNodeAt(i32 x, i32 y) const
        {
            if (IsValidPos(x, y) == false)
                return false;

            return _nodes[ConvertToNodeIdx(x, y)].state == NodeState::Walkable;
        }

        const std::vector<AdjacentNode>& GetAdjacentNodesAt(i32 x, i32 y)
        {
            if (IsValidPos(x, y) == false)
                return kNullAdjNodes;

            // { x, y }와 인접한 이동 가능한 노드 목록을 가져온다.
            return _adjNodeTables[(i32)ConvertToNodeIdx(x, y)];
        }

        // 이동 가능성을 확인하기 위한 용도의 함수
        // dx : -1, 0, 1
        // dy : -1, 0, 1
        // bool IsWalkableFromHereToDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        bool CanMoveByDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        {
            AdjacentDirection dir = ConvertToNodeDirection(dx, dy);

            // 방향 검증
            if (dir == AdjacentDirection::InvalidDirection)
                return false;

            // 위치 검증
            if (IsValidPos(hereX, hereY) == false)
                return false;

            // 노드 조회
            i32 nodeIdx = (i32)ConvertToNodeIdx(hereX, hereY);

            return _neighborTables[nodeIdx][(i32)dir];
        }

        // 이동 가능한 노드를 조회하기 위한 용도의 함수
        // dx : -1, 0, 1
        // dy : -1, 0, 1
        // const GridNode& GetWalkableNodeFromDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        // const GridNode& CanMoveByDeltaPosNode(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        // {
        //     AdjacentDirection dir = ConvertToNodeDirection(dx, dy);
        // 
        //     // 방향 검증
        //     if (dir == AdjacentDirection::InvalidDirection)
        //         return kNullNodeInvalidDirection;
        // 
        //     // 위치 검증
        //     if (IsValidPos(hereX, hereY) == false || IsValidPos(hereX + dx, hereY + dy) == false)
        //         return kNullNodeOutOfBounds;
        // 
        //     // 노드 조회
        //     i32 nodeIdx = (i32)ConvertToNodeIdx(hereX + dx, hereY + dy);
        // 
        //     if (_neighborTables[nodeIdx][(i32)dir] == false)
        //         return kNullNodeNotWalkable;
        // 
        //     return _nodes[nodeIdx];
        // }

    public:
        /*************************
        *      For JPS Plus      *
        *************************/
        bool IsOptimizedForJpsPlus() const
        {
            return _isOptimizedForJpsPlus;
        }
        
        const JumpPointNode& GetJumpPointNodeAt(i32 x, i32 y) const
        {
            if (_isOptimizedForJpsPlus == false)
                return kNullJumpPointNode;

            if (IsValidPos(x, y) == false)
                return kNullJumpPointNode;

            return _jumpPointNodes[(i32)ConvertToNodeIdx(x, y)];
        }

        const std::vector<AdjacentDirection>& GetJumpDirLookUpTable(AdjacentDirection fromDir) const
        {
            if (_isOptimizedForJpsPlus == false)
                return kNullJumpTable;

            // 나아갈 수 있는 가능성을 가진 방향을 묶은 배열을 반환한다.
            return _jumpDirLookUpTables[(i32)fromDir];
        }

    private:
        /*************************
        *      Optimization      *
        *************************/
        /**
         * optimize()는 Create()가 호출될 때 같이 호출될 함수
         * Create()의 인자로 전달한 allowDiagonal과 allowCorners에 영향을 받는다.
         * 
         * allowDiagonal : 대각선 방향의 이웃 노드 설정 여부
         * allowCorners  : 대각선 방향으로 이동 시 마주하는 코너를 통과할 것인지에 대한 여부
         */
        void optimize();

        /*************************
        *      For JPS Plus      * | JPS+는 allowDiagonal와 allowCorners가 둘 다 true여야 함.
        *************************/
        // 아래 함수 목록은 OptimizeForJpsPlus() 과정에서 사용된다.
        void prepareJumpLookUpTable();

        // 맵에서 강제 이웃을 찾기 위한 용도의 함수
        void preprocessPrimaryJumpPoints();

        // 아래 2개의 함수는 JP 간 이동 거리를 포함해 Wall Distance도 함께 계산한다.
        void preprocessStraightJumpPoints();
        void preprocessDiagonalJumpPoints();

    private:
        /*********************
        *      Map Info      *
        *********************/
        i32 _width  = 0;
        i32 _height = 0;

        std::vector<GridNode> _nodes;

    private:
        /********************
        *      Options      *
        ********************/
        bool _isAllowedDiagonal = true;
        bool _isAllowedCorners  = true;

    private:
        /*************************
        *      Optimization      *
        *************************/
        std::vector<Vec2D<i32>> _blockedPositions; // 갈 수 없는 모든 위치를 담음.

        /**
         * 노드의 인덱스를 토대로 방향 쪽으로 인접한 노드가 있는지를 판별한다.
         * auto& neighborTable = _neighborTables[ConvertToNodeIdx(x, y)];
         * 
         * for (i32 idx = 0; idx < (i32)AdjacentDirection::NumDirections; idx++)
         * {
         *     if (connection.neighbors[idx] == false)
         *         continue;
         * 
         *     ...
         * }
         */
        std::vector<NeighborTable> _neighborTables;

        /**
         * 노드의 인덱스를 기반으로 인접한 노드를 즉각 가져오기 위해 사용한다.
         * 해당 방식을 이용하면 인접한 영역이 유효한지 탐색하지 않아도 된다.
         * 
         * for (auto [node, dir] : _adjNodeTables[ConvertToNodeIdx(x, y)])
         * {
         *     ...
         * }
         */
        std::vector<std::vector<AdjacentNode>> _adjNodeTables;

        /*************************
        *      For JPS Plus      *
        *************************/
        bool _isOptimizedForJpsPlus = false;

        std::vector<JumpPointNode> _jumpPointNodes;

        // 나아갈 수 있는 가능성을 가진 방향을 묶어서 배열로 표현하기 위한 변수
        // ex 1) NorthEast -> North + NorthEast + East
        // ex 2) North -> West + NorthWest + North + NorthEast + East
        std::vector<AdjacentDirection> _jumpDirLookUpTables[(i32)AdjacentDirection::NumDirections];

        // _jumpDirLookUpTables을 통해 나아갈 방향을 구성할 때 AdjacentDirection 대신 JpsDirectionFlags를 사용해도 된다.
        // 하지만 이 경우 2^8(AllFlags + 1) 만큼 배열을 할당해서 플래그가 세워질 수 있는 모든 경우의 수를 대상으로 나아갈 방향을 세팅해야 한다.
        // 
        // JPS+의 원서를 보면 방향 테이블을 쓰는 것 같으니 AdjacentDirection을 쓰도록 하자.

    public:
        /*************************
        *      Null Objects      *
        *************************/
        static constexpr GridNode kNullNodeOutOfBounds{ -1, -1, NodeState::Invalid };
        static constexpr GridNode kNullNodeInvalidDirection{ -1, -1, NodeState::Invalid };
        static constexpr GridNode kNullNodeNotWalkable{ -1, -1, NodeState::Invalid };

        const std::vector<AdjacentNode> kNullAdjNodes;

        static constexpr JumpPointNode kNullJumpPointNode{ };
        const std::vector<AdjacentDirection> kNullJumpTable;
    };
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
