// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Map:DynamicMap;

import :Node;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;

    /**
     * DynamicMap -> StaticMap -> PathFinding
     * 
     * DynamicMap은 사용자의 요구에 맞게 맵에 있는 노드 정보를 갱신한다.
     * 실제 길찾기를 수행할 때는 정적인 정보인 StaticMap을 사용해야 한다(최적화를 겸함).
     * 이 두 유형의 맵은 정보를 제공할 뿐이며 실제 길찾기 과정은 수행하지 않는다.
     * 
     * 실제 길찾기를 수행할 때는 PathFinding::Algorithm에 있는 것을 써야 한다.
     * 맵 정보와 길찾기 알고리즘을 디커플링하기 위해서 이렇게 작성한 것이다.
     * 
     * 비슷한 이유로 맵 정보를 토대로 맵을 그리는 작업은 MapView에서 진행해야 한다.
     */
    class DynamicMap final
    {
    private:
        struct Deleter
        {
            void operator()(DynamicMap* instance) const;
        };

    private:
        /*************************
        *      Rule of Five      *
        *************************/
        explicit DynamicMap() = default; // constructor
        ~DynamicMap() = default; // destructor

        DynamicMap(const DynamicMap& rhs) = delete; // copy constructor
        DynamicMap& operator=(const DynamicMap& rhs) = delete; // copy assignment

        DynamicMap(DynamicMap&& rhs) noexcept = delete; // move constructor
        DynamicMap& operator=(DynamicMap&& rhs) noexcept = delete; // move assignment
    
    public:
        static std::shared_ptr<DynamicMap> Create();
        static std::shared_ptr<DynamicMap> Create(i32 width, i32 height);
        static std::shared_ptr<DynamicMap> Create(std::shared_ptr<StaticMap>& staticMap);
        // static std::shared_ptr<DynamicMap> Create(std::string_view filename);

    public:
        std::shared_ptr<DynamicMap> Clone() const;

    public:
        void Clear(); // 모든 노드를 움직일 수 있는 상태로 바꿈.
        void Resize(i32 width, i32 height);

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

        void SetNodeStateAt(i32 x, i32 y, NodeState state)
        {
            if (IsValidPos(x, y) == false)
                return;

            _nodes[ConvertToNodeIdx(x, y)].state = state;
        }

        bool IsWalkableNodeAt(i32 x, i32 y) const
        {
            if (IsValidPos(x, y) == false)
                return false;

            return _nodes[ConvertToNodeIdx(x, y)].state == NodeState::Walkable;
        }

    private:
        i32 _width  = 0;
        i32 _height = 0;

        std::vector<GridNode> _nodes;

    public:
        /*************************
        *      Null Objects      *
        *************************/
        static constexpr GridNode kNullNodeOutOfBounds{ -1, -1, NodeState::Invalid };
    };
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
