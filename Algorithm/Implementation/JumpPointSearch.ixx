// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:JumpPointSearch;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Jump Point Search
     * 
     * https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search/
     * https://harablog.wordpress.com/2011/09/07/jump-point-search/
     * https://www.gamedev.net/articles/programming/artificial-intelligence/jump-point-search-fast-a-pathfinding-for-uniform-cost-grids-r4220/
     * 
     * Jump Point�� Ž���ϴ� ������� �����ϴ� ��ã�� �˰����̸�
     * ���� �� ���� ��Ҵ� JPS�� �ٽ��� �Ǵ� �κ��̴�.
     * 
     * 1. ����ġ��(Pruning)
     *  - A*ó�� ��� ������ Ž���ϴ� ���� �ƴ� ���ʿ��� ������ �����ϰ� Ư�� �������� Ž���ϴ� ����� ����.
     *  - �̸� ���� OpenSet�� ��带 �ִ� ������ �ּ�ȭ�� �� ����.
     *  - ����ġ�⸦ ���� Ž���� ���� �̿��� ������ ������ ��ӵ�.
     *
     * 2. ���� �̿�(Forced Neighbor)
     *  - ���� �ִ� ��ο� �����ϱ� ���� �湮 �� ��ȸ�ؾ� �ϴ� �̿��� ����.
     *  - ���� �ڳ� �������� �߻��ϸ� ���� �̿��� Ž���ϸ� ���� ���������� Ž���� ���߰� Ž�� ������ ���� �̿����� ������.
     *  - ���� �̿��� ��ġ�� �ִ� ���� Jump Point�� ���Ǿ�� �ϱ⿡ OpenSet�� ��.
     * 
     * 3. Jump Point
     *  - �ǳʶپ Ž���� �簳�ϴ� ������ ����.
     *  - ���� ����� ���� ���� ��ȸ�ϱ� ���� ����� ���� ���������� Ž���� �̾ ������.
     * 
     * (����) ���� �̿��� Jump Point�� �ƴϸ�, �̴� Jump Point�� �� �� �ִ� ���ɼ��� ���� �ĺ��� ���Ѵ�(��� Forced Neighbor�� JP�� ���� �ƴ�).
     * 
     * JPS�� �� ���������� Ž�� �������� �߻��ϴ� ���ʿ��� OpenSet������ ���� ������ �����Ͽ� Ž���� ����ȭ�� �˰����̴�.
     * JPS ��ü�� 8���� Ž���� �⺻ ���ڷ� �ϰ� �ֱ� ������ �밢�� �̵��� ����ϴ� ���� ����.
     * ���� ����ġ�� ������ ����ȭ�ϱ� ���� �ڳ� �̵� ���θ� Ȯ���ϰ� ���ϰ� ������ �Ѵ�.
     * 
     * ���� �밢�� �̵��̳� �ڳ� �̵��� ������� �ʴ´ٸ� ������ Ž�� �Լ��� ���� �����ؾ� �� ���� �ִ�.
     * �밢�� + �ڳ� ���, �밢�� + �ڳ� �����, �밢�� ����� ���� Jump Point ���� ������ �����ϴ�.
     */
    
    // ����� �����ϴ� ��� ����
    // ��� ã�� �������� std::unordered_map ���
    bool JumpPointSearch1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
    bool JumpPointSearch2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 20% ���� �� ������)
    bool JumpPointSearch3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
    // 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ���� ������ �� ����.
    bool JumpPointSearch4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                          HeuristicType heuristicType, f32 heuristicWeight,
                          std::vector<Vec2D<i32>>* outResult);

    
    // PathFindingContext�� �̿��� Ž���� �����ϴ� ���
    bool JumpPointSearchAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool JumpPointSearchComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
