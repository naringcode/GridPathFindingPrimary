// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:BreadthFirstSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Breadth-First Search(�ʺ� �켱 Ž��)
     * 
     * �ʺ� Ȯ���ϸ� Ž���ϴ� ����� ���ϸ� �ʺ� ������ FIFO(First-In First-Out) ������� �����Ѵ�.
     */
    
    // ����� �����ϴ� ��� ����
    // ��� ã�� �������� std::unordered_map ���
    bool BreadthFirstSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
    bool BreadthFirstSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 20% ���� �� ������)
    bool BreadthFirstSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
    // 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ���� ������ �� ����.
    bool BreadthFirstSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext�� �̿��� Ž���� �����ϴ� ���
    bool BreadthFirstSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool BreadthFirstSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
