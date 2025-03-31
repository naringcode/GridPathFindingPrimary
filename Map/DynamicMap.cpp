// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.Map:DynamicMap;

import :StaticMap;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

void DynamicMap::Deleter::operator()(DynamicMap* instance) const
{
    if (instance == nullptr)
        return;

    instance->_width  = 0;
    instance->_height = 0;

    instance->_nodes.clear();

    delete instance;
}

std::shared_ptr<DynamicMap> DynamicMap::Create()
{
    DynamicMap* instance = new DynamicMap{ };

    instance->_width  = 0;
    instance->_height = 0;

    instance->_nodes.clear();

    return std::shared_ptr<DynamicMap>{ instance, Deleter{ } };
}

std::shared_ptr<DynamicMap> DynamicMap::Create(i32 width, i32 height)
{
    DynamicMap* instance = new DynamicMap{ };

    instance->_width  = width;
    instance->_height = height;

    instance->_nodes.resize(width * height);

    // Legacy Code
    // for (i32 y = 0; y < height; y++)
    // {
    //     for (i32 x = 0; x < width; x++)
    //     {
    //         i32 idx = x + (y * width);
    // 
    //         instance->_nodes[idx] = { x, y, NodeState::Walkable };
    //     }
    // }

    // C++23이라면 cartesian_product_view와 zip_view를 연계해서 더 쉽게 코드를 작성할 수 있다.
    std::ranges::generate(instance->_nodes, [&, idx = 0]() mutable {
        i32 nodeIdx = idx++;

        i32 x = nodeIdx % width;
        i32 y = nodeIdx / width;

        return GridNode{ x, y, NodeState::Walkable };
    });

    return std::shared_ptr<DynamicMap>{ instance, Deleter{ } };
}

std::shared_ptr<DynamicMap> DynamicMap::Create(std::shared_ptr<StaticMap>& staticMap)
{
    if (staticMap == nullptr)
        return nullptr;

    DynamicMap* instance = new DynamicMap{ };
    
    instance->_width  = staticMap->GetWidth();
    instance->_height = staticMap->GetHeight();
    
    instance->_nodes.resize(instance->_width * instance->_height);

    // Legacy Code
    // for (i32 y = 0; y < instance->_height; y++)
    // {
    //     for (i32 x = 0; x < instance->_width; x++)
    //     {
    //         i32 idx = x + (y * instance->_width);
    // 
    //         instance->_nodes[idx] = staticMap.GetNodeAt(x, y);
    //     }
    // }

    // C++23이라면 cartesian_product_view와 zip_view를 연계하여 더 쉽게 코드를 작성할 수 있다.
    std::ranges::generate(instance->_nodes, [&, idx = 0]() mutable {
        i32 nodeIdx = idx++;
    
        i32 x = nodeIdx % instance->_width;
        i32 y = nodeIdx / instance->_width;
    
        return staticMap->GetNodeAt(x, y);
    });
    
    return std::shared_ptr<DynamicMap>{ instance, Deleter{ } };
}

std::shared_ptr<DynamicMap> DynamicMap::Clone() const
{
    DynamicMap* instance = new DynamicMap;

    instance->_width  = _width;
    instance->_height = _height;

    instance->_nodes.resize(_width * _height);

    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         i32 idx = x + (y * _width);
    // 
    //         instance->_nodes[idx].state = _nodes[idx].state;
    //     }
    // }

    std::ranges::copy(_nodes, instance->_nodes.begin());

    return std::shared_ptr<DynamicMap>{ instance, Deleter{ } };
}

void DynamicMap::Clear()
{
    // Legacy Code
    // for (i32 y = 0; y < _height; y++)
    // {
    //     for (i32 x = 0; x < _width; x++)
    //     {
    //         i32 idx = x + (y * _width);
    // 
    //         _nodes[idx].state = NodeState::Walkable;
    //     }
    // }

    std::ranges::for_each(_nodes, [](GridNode& node) {
        node.state = NodeState::Walkable;
    });
}

void DynamicMap::Resize(i32 width, i32 height)
{
    std::vector<GridNode> newNodes;

    newNodes.resize(width * height);

    // Legacy Code
    // for (i32 y = 0; y < height; y++)
    // {
    //     for (i32 x = 0; x < width; x++)
    //     {
    //         i32 oldIdx = x + (y * _width);
    //         i32 newIdx = x + (y * width);
    // 
    //         newNodes[newIdx] = { x, y, NodeState::Walkable };
    // 
    //         // 이전 영역의 정보 계승
    //         if (x < _width && y < _height)
    //         {
    //             newNodes[newIdx].state = _nodes[oldIdx].state;
    //         }
    //     }
    // }

    std::ranges::for_each(newNodes, [&, idx = 0](GridNode& node) mutable {
        i32 nodeIdx = idx++;

        i32 x = nodeIdx % width;
        i32 y = nodeIdx / width;

        // 이전 영역의 정보 계승
        if (x < _width && y < _height)
        {
            i32 prevIdx = x + (y * _width);

            node = { x, y, _nodes[prevIdx].state };
        }
        else
        {
            node = { x, y, NodeState::Walkable };
        }
    });

    _width  = width;
    _height = height;

    _nodes = std::move(newNodes);
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
