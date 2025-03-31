// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:AStarSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * A Star
     * 
     * f(n) = g(n) + h(n)
     * �̵� �Ÿ��� �޸���ƽ ���� ȥ���Ͽ� ���� ��ġ�� ���� ������ �򰡵Ǵ� ��带 �켱������ Ž���ϴ� ����� ����Ѵ�.
     */

    // ����� �����ϴ� ��� ����
    // ��� ã�� �������� std::unordered_map ���
    bool AStarSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
    bool AStarSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 20% ���� �� ������)
    bool AStarSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
    // 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ���� ������ �� ����.
    bool AStarSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                      HeuristicType heuristicType, f32 heuristicWeight,
                      std::vector<Vec2D<i32>>* outResult);

    // PathFindingContext�� �̿��� Ž���� �����ϴ� ���
    bool AStarSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool AStarSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
