// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:DepthFirstSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Depth-First Search(���� �켱 Ž��)
     * 
     * ���� ���� ������ �ִ� ��带 �켱������ Ž���ϴ� ����̴�.
     * FILO(First-In Last-Out) ������� �����ϱ� ������ Stack ������� ó���ؾ� �Ѵ�.
     * ������ ����� Ž�� ������� �����ϴ� ��찡 ������ ���⼱ Ž�� �Ÿ��� �켱���� ť�� �ִ� ����� ����� ���̴�.
     * 
     * ���� ������ Ž���ϴ� ������� ���� �������� ������ ��带 �켱������ ó���Ѵ�. 
     */

    // ����� �����ϴ� ��� ����
    // ��� ã�� �������� std::unordered_map ���
    bool DepthFirstSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
    bool DepthFirstSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 20% ���� �� ������)
    bool DepthFirstSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
    // 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ���� ������ �� ����.
    bool DepthFirstSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos, std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext�� �̿��� Ž���� �����ϴ� ���
    bool DepthFirstSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool DepthFirstSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
