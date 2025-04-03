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
     * JPS+�� ����ϱ� ���� ������ �����ϴ� ��ĵ� �����ϰ� �ִ�.
     * �˰��� ���� ����� JPS+ ���� �ڵ�� ������ �����ϵ��� �Ѵ�.
     * 
     * ���� �������� JPS+�� �ڳʸ� ������� �ʱ⿡ �̸� ����Ϸ��� ���� �̿��� ����ϴ� ����� �ٲ�� �Ѵ�.
     */
    /**
     * flood-fill �˰����� �Ἥ �� �� ���� ������ �����ϴ� ����� �ϴ��� �����ϵ��� �Ѵ�(���� ������ ���� �ƴ�).
     * 
     * DynamicMap -> StaticMap -> PathFinding
     * 
     * StaticMap�� ���� ������ ������ ���� �����ϱ� ������ ����ȭ�ؼ� ����ص� �ȴ�.
     * ���� DynamicMap�� ���������� ��ã�⿡ ���� ������ �����ϱ� �ص� ���� ��ã�� ������ �������� �ʴ´�.
     */
    class StaticMap final
    {
    public:
        // ��� ������ ��ȸ�Ͽ� �̵� ������ ���� ��尡 �����ϴ��� Ȯ���ϱ� ���� ���̺�
        using NeighborTable = std::array<bool, (i32)AdjacentDirection::NumDirections>;

        // �̵� ������ ����� ���� ������ ǥ���ϱ� ���� ����ü
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
            // Primary Jump Point�� ���� ���� ���� �÷��װ� ����ȴ�(JpsDirectionFlags).
            i32 primaryIndegreeFlags = 0;

            // 1. ���� ������ ������ Jump Point���� �Ÿ��� ����ȴ�.
            // 2. ���� �������� Jump Point�� ��ȸ�� �� ������ ������ �Ÿ��� ������ ����ȴ�.
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
        bool OptimizeForJpsPlus(); // JPS+�� ����ϱ� ���ؼ� �ش� �Լ��� ������ ȣ���ؾ� ��.

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

            // { x, y }�� ������ �̵� ������ ��� ����� �����´�.
            return _adjNodeTables[(i32)ConvertToNodeIdx(x, y)];
        }

        // �̵� ���ɼ��� Ȯ���ϱ� ���� �뵵�� �Լ�
        // dx : -1, 0, 1
        // dy : -1, 0, 1
        // bool IsWalkableFromHereToDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        bool CanMoveByDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        {
            AdjacentDirection dir = ConvertToNodeDirection(dx, dy);

            // ���� ����
            if (dir == AdjacentDirection::InvalidDirection)
                return false;

            // ��ġ ����
            if (IsValidPos(hereX, hereY) == false)
                return false;

            // ��� ��ȸ
            i32 nodeIdx = (i32)ConvertToNodeIdx(hereX, hereY);

            return _neighborTables[nodeIdx][(i32)dir];
        }

        // �̵� ������ ��带 ��ȸ�ϱ� ���� �뵵�� �Լ�
        // dx : -1, 0, 1
        // dy : -1, 0, 1
        // const GridNode& GetWalkableNodeFromDeltaPos(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        // const GridNode& CanMoveByDeltaPosNode(i32 hereX, i32 hereY, i32 dx, i32 dy) const
        // {
        //     AdjacentDirection dir = ConvertToNodeDirection(dx, dy);
        // 
        //     // ���� ����
        //     if (dir == AdjacentDirection::InvalidDirection)
        //         return kNullNodeInvalidDirection;
        // 
        //     // ��ġ ����
        //     if (IsValidPos(hereX, hereY) == false || IsValidPos(hereX + dx, hereY + dy) == false)
        //         return kNullNodeOutOfBounds;
        // 
        //     // ��� ��ȸ
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

            // ���ư� �� �ִ� ���ɼ��� ���� ������ ���� �迭�� ��ȯ�Ѵ�.
            return _jumpDirLookUpTables[(i32)fromDir];
        }

    private:
        /*************************
        *      Optimization      *
        *************************/
        /**
         * optimize()�� Create()�� ȣ��� �� ���� ȣ��� �Լ�
         * Create()�� ���ڷ� ������ allowDiagonal�� allowCorners�� ������ �޴´�.
         * 
         * allowDiagonal : �밢�� ������ �̿� ��� ���� ����
         * allowCorners  : �밢�� �������� �̵� �� �����ϴ� �ڳʸ� ����� �������� ���� ����
         */
        void optimize();

        /*************************
        *      For JPS Plus      * | JPS+�� allowDiagonal�� allowCorners�� �� �� true���� ��.
        *************************/
        // �Ʒ� �Լ� ����� OptimizeForJpsPlus() �������� ���ȴ�.
        void prepareJumpLookUpTable();

        // �ʿ��� ���� �̿��� ã�� ���� �뵵�� �Լ�
        void preprocessPrimaryJumpPoints();

        // �Ʒ� 2���� �Լ��� JP �� �̵� �Ÿ��� ������ Wall Distance�� �Բ� ����Ѵ�.
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
        std::vector<Vec2D<i32>> _blockedPositions; // �� �� ���� ��� ��ġ�� ����.

        /**
         * ����� �ε����� ���� ���� ������ ������ ��尡 �ִ����� �Ǻ��Ѵ�.
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
         * ����� �ε����� ������� ������ ��带 �ﰢ �������� ���� ����Ѵ�.
         * �ش� ����� �̿��ϸ� ������ ������ ��ȿ���� Ž������ �ʾƵ� �ȴ�.
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

        // ���ư� �� �ִ� ���ɼ��� ���� ������ ��� �迭�� ǥ���ϱ� ���� ����
        // ex 1) NorthEast -> North + NorthEast + East
        // ex 2) North -> West + NorthWest + North + NorthEast + East
        std::vector<AdjacentDirection> _jumpDirLookUpTables[(i32)AdjacentDirection::NumDirections];

        // _jumpDirLookUpTables�� ���� ���ư� ������ ������ �� AdjacentDirection ��� JpsDirectionFlags�� ����ص� �ȴ�.
        // ������ �� ��� 2^8(AllFlags + 1) ��ŭ �迭�� �Ҵ��ؼ� �÷��װ� ������ �� �ִ� ��� ����� ���� ������� ���ư� ������ �����ؾ� �Ѵ�.
        // 
        // JPS+�� ������ ���� ���� ���̺��� ���� �� ������ AdjacentDirection�� ������ ����.

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
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
