// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:JumpPointSearchPlus;

import :Core;
import :Heuristic;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;
    class PathFindingContext;

    /**
     * Jump Point Search Plus
     *
     * https://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter14_JPS_Plus_An_Extreme_A_Star_Speed_Optimization_for_Static_Uniform_Cost_Grids.pdf
     * https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter17_Pathfinding_Architecture_Optimizations.pdf
     * https://www.gameaipro.com/GameAIPro3/GameAIPro3_Chapter22_Faster_A_Star_with_Goal_Bounding.pdf
     * https://github.com/SteveRabin/JPSPlusWithGoalBounding/blob/master/Rabin_AISummitGDC2015_JPSPlusWithGoalBounding.pptx
     * 
     * JPS�� ������ ������ ������ �˰����̴�.
     * JPS�� ��Ÿ�� ���� Jump Points�� �����ߴٸ� JPS+�� ������ ����� ���̺��� ���� Jump Points�� ã�´�.
     * 
     * JPS���� Ž���� �����ϴ� ���� ���� Forced Neighbors�� �߻��ϴ� ��ġ�� ������ �ʴ´�.
     * ��, �̷��� ��ġ�� ������ �̸� �ľ��ؼ� Jump Points�� ��ġ�� �̸� ����ϴ� ��(Baking)�� �����ϴ�.
     * 
     * JPS�� ��Ÿ�� �� Jump Point�� �����ߴٸ� JPS+�� �̰� ������ �̸� ����� ���̺��� ����Ѵ�.
     * JPS+�� JPS�� Jump Point �� Ž�� ������ ������ ����Ͽ� ����(?)�ϴ� ������ ����ȭ�ϴ� ����̶�� ���� �ȴ�.
     * 
     * --------------------------------------------------
     * 
     * JPS+�� ���� 4���� ������ Jump Points�� ����Ѵ�.
     * 1. Primary Jump Points(������ ���� ��)
     * 2. Straight Jump Points(������ ���� ��)
     * 3. Diagonal Jump Points(������ ���� ��)
     * 4. �� Target Jump Points(��Ÿ�� ���߿� ����Ͽ� ����) ��
     * 
     * 
     * ������ ����Ͽ� Jump Points�� �����ϴ� ����� �̷��ϴ�.
     * 
     * 1. ���� �ִ� ��η� �����ϱ� ���� Forced Neighbors�� ��ġ�� �̸� �ľ��Ѵ�.
     * 
     * 2. �� Forced Neighbor�� "������ �� �ִ� ����"�� ǥ���ϰ� �̸� Primary Jump Point��� �Ѵ�.
     *  - �˻� ���� �ش� �������� �����ؾ� Primary Jump Point�� �Ǵ� ����.
     *  - ���� ��带 �湮�Ѵٰ� �ص� "������ �� �ִ� ����"���� ������ ������ Primary Jump Point�� �� �� ����.
     *  - Primary Jump Point�� Cardinal Direction �������� �߻��ϱ� ������ 4���� Flags�� ǥ���� �� ����.
     * 
     * #. Forced Neighbors�� ã�Ƴ��� �������� Primary Jump Points�� ����� ���� ȿ�����̴�.
     * 
     * 3. Primary Jump Points�� ���� ����� �������� �� ������ Straight Jump Points�� ����ؾ� �Ѵ�.
     *  - Straight Jump Point�� �����¿� ������ Ž������ �� �����ϰ� �Ǵ� Primary Jump Point���� �Ÿ����� ���Ե�.
     *  - Primary Jump Point�� ������ �Ǵ� ������ �ƴ� "������ �� �ִ�" �����̾�� ��.
     *  - (�߿�) Straight Jump Point ���� ���� ����� Primary Jump Point���� �Ÿ��� �ǹ��ϴ� ���� �ƴ�.
     * 
     * #. Primary Jump Point�� Straight Jump Point�� ������ ���̴�.
     *  - ������ ����� Ư�� ��ġ�� �̷��� ������ Ư���� ���ÿ� ǥ���� �� �־�� ��.
     * 
     * 4. Primary Jump Points�� Straight Jump Points�� ����� �������� �밢�� ������ Jump Points�� Diagonal Jump Points�� ����ؾ� �Ѵ�.
     *  - �̰� �밢�� ������ ���� ���а� ���õ� ����(�� = �� + ��)���� �����ϰ� �Ǵ� Straight Jump Point�� ������ ����.
     *   - Straight Jump Point ��ü�� Primary Jump Point���� ���� ����� �������� ����.
     * 
     * #. �� �κ��� �����ϱⰡ ������ �� �ִ�.
     *  - �ϵ���(��)���� �̵��� �� �ִ� Diagonal Jump Point�� ���� ��尡 �Ǳ� ���ؼ�?
     *   - ���� �밢 ��ġ�� �������� �� ������ ����(��)�̳� ����(��) ���������� Straight Jump Point�� ������ �� �־�� ��.
     *   - �̰� JPS�� �����ϴ� �밢�� �����̿� ������ �ſ� ������.
     *   - ���� �밢 ������ ���� Straight Jump Point�� ȯ�� �����ϴٸ� �� �Ÿ��� ���ԵǾ�� ��.
     *  - �밢�� �������� Ž���� �����ϸ鼭 Cardinal ���������� ȯ�� ������ ã�� �����̶�� �����ϸ� ��.
     * 
     * 5. Jump Points �� ����ִ� �κ��� ������ �Ÿ��� Wall Distances�� ä������.
     *  - �� ���� Straight Jump Points�� Diagonal Jump Points�� ����� �� �Բ� �ۼ���.
     *  - ���� Jump Points���� ���� �����ϱ� ���� 0�� ���̳ʽ� ������ ǥ���.
     *  - ������ ����� ���� ���� ��ȣ�� ������ ��.
     * 
     * 
     * �̷��� ���� ��ó���ϸ� Ž���� �ʿ��� ���� ������ ������ ������ �� �ִ�.
     * �̷и� ������ Jump Point�� ��Ÿ�� ���� ������ �ʿ䰡 ���� ������ JPS���� ������.
     * !! �׽�Ʈ�غ��� ���� OpenSet�� ���� ����� ������ ���ؼ� ������ �����Ǵ� ��찡 ����. !!
     * 
     * 
     * �� Target Jump Point�� 4���� ������ Jump Points �� �����ϰ� ��Ÿ�� �� �����Ǵ� Jump Point�̴�.
     * - ���� �� ���� ���� Ž������ Target Jump Point�� �����ϴ� �Ͱ� �밢�� ���� Ž������ Target Jump Point�� �����ϴ� ���� �ٸ�.
     * - ���� �� ���� ���� Ž������ Target Jump Point�� ���� �Ÿ� �̳��� ��ǥ�� ���� ��쿡�� �����ȴ�.
     * 
     * #. �밢�� ���� Ž�������� Target Jump Point ������ �ټ� �����ϴ�.
     * - �۾� ��ġ �������� ��ǥ ��ġ������ ���ο� ���� �Ÿ��� �����غ����� ��,
     *   �� �� �ִ� �Ÿ��� �밢�� ���������� �� �Ǵ� ���� ���� ���������� �Ÿ����� ª�ų� ������ ������.
     * - �̴� ��Ÿ�� ���� ���� ȯ�� ����(Target Jump Point)�� ����� ���������� �����ϴ� ���� �õ��ϴ� ���̶� ���� ��.
     * 
     * ���⼭ �����ؾ� �ϴ� ���� �ִµ� �밢�� ���������� Target Jump Point�� ���� ������ ������ �� ��� �ϴ� �����ǰ� ���ٴ� ���̴�.
     * �밢�� �������� �̵� �� ���� �� ���� ������ ���� ���� ������ ������ �� ���� ������ "����"�ȴٸ� �ϴ� Target JP�� �����ϰ� ����.
     * 
     * #. �밢�� ���� Ž�������� Target Jump Point ���� ������ ���������� ���� ������ ���� Jump Point�� �����Ѵ�.
     *  1. ��ǥ ���������� ���� �Ÿ��� ���� �Ÿ� �� �ּڰ��� ����.
     *  2. �� ũ�⸸ŭ �밢�� �������� �̵��ϴ� Jump Point�� ������.
     * 
     * �̶� �밢�� Ž�� ���⿡ ��ǥ ������ �ִٸ�?
     *  - ���ο� Target Jump Point�� ��ġ�� ��ǥ ����� ��ġ�� ������.
     * 
     * ������ ���������� ���� ������ �ڵ带 ���� ���� ������ �� �ִ�.
     * ���� ��ó�� Ȥ�� Baking�ϴ� ������ �̸� ���� Ž���ϴ� ���� ��ü�� ������ ������ �����ؼ� ���� �Ѵ�.
     */

    // ����� �����ϴ� ��� ����
    // ��� ã�� �������� std::unordered_map ���
    bool JumpPointSearchPlus1(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map ���(1�� ��ĺ��� ������ �뷫 15% ���� �� ������)
    bool JumpPointSearchPlus2(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::unordered_map�� Ž�� ���̵� ����(2�� ��ĺ��� ������ �뷫 20% ���� �� ������)
    bool JumpPointSearchPlus3(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);

    // ��� ã�� �������� thread_local std::vector�� Ž�� ���̵� ���(�̰� �Ź� ��带 ���� ũ�⸸ŭ �Ҵ��� �� ���� ������ thread_local�� �ʼ���)
    // 3�� ��ĺ��� ������ �뷫 20% ���� �� ������ 1�� ��İ� ���ϸ� ���� 40% ���� ������ �� ����.
    bool JumpPointSearchPlus4(std::shared_ptr<StaticMap>& staticMap, const Vec2D<i32>& startPos, const Vec2D<i32>& destPos,
                              HeuristicType heuristicType, f32 heuristicWeight,
                              std::vector<Vec2D<i32>>* outResult);
    
    // PathFindingContext�� �̿��� Ž���� �����ϴ� ���
    bool JumpPointSearchPlusAdvance(std::shared_ptr<PathFindingContext>& pathFindingContext);
    bool JumpPointSearchPlusComplete(std::shared_ptr<PathFindingContext>& pathFindingContext);
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
