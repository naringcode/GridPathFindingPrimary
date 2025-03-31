// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:Heuristic;

import :Core;

// Module Purview / Module Interface : Optional
namespace PathFinding
{
    /**********************
    *      Manhattan      * | �׽�Ʈ�غ��� �� ����� �ణ ����Ȯ�ϱ� �ص� ���� ������ ��θ� ã�Ҵ� ���
    **********************/
    f32 HeuristicManhattan(f32 dx, f32 dy)
    {
        return abs(dx) + abs(dy);
    }

    f32 HeuristicManhattan(f32 sx, f32 sy, f32 ex, f32 ey)
    {
        return HeuristicManhattan(sx - ex, ey - sy);
    }

    /**********************
    *      Euclidean      *
    **********************/
    f32 HeuristicEuclidean(f32 dx, f32 dy)
    {
        return sqrt(dx * dx + dy * dy);
    }

    f32 HeuristicEuclidean(f32 sx, f32 sy, f32 ex, f32 ey)
    {
        return HeuristicEuclidean(sx - ex, ey - sy);
    }

    /*******************
    *      Octile      * | �Ϲ������� 8���� ��Ŀ��� ��õ�Ǵ� �޸���ƽ ���(����ġ�� 1.5 ������ �ָ� �ӵ��� ������ ǰ���� ������ ����)
    *******************/
    f32 HeuristicOctile(f32 dx, f32 dy)
    {
        dx = abs(dx);
        dy = abs(dy);

        if (dx > dy)
        {
            return (kSqrt2 * dy) + (dx - dy);
        }
        else
        {
            return (kSqrt2 * dx) + (dy - dx);
        }
    }

    f32 HeuristicOctile(f32 sx, f32 sy, f32 ex, f32 ey)
    {
        return HeuristicOctile(sx - ex, ey - sy);
    }

    /**********************
    *      Chebyshev      *
    **********************/
    f32 HeuristicChebyshev(f32 dx, f32 dy)
    {
        return std::max(abs(dx), abs(dy));
    }

    f32 HeuristicChebyshev(f32 sx, f32 sy, f32 ex, f32 ey)
    {
        return HeuristicChebyshev(sx - ex, ey - sy);
    }
}

export namespace PathFinding
{
    // ���ϴ� �޸���ƽ ������ ������ �˾Ƽ� �߰��ϵ��� �Ѵ�.
    enum class HeuristicType
    {
        Manhattan,
        Euclidean,
        Octile,
        Chebyshev,

        Max
    };

    struct HeuristicTypeInfo
    {
        const HeuristicType type{ };
        std::string_view name{ };
    };

    const std::array<HeuristicTypeInfo, (i32)HeuristicType::Max> kHeuristicTypeInfoList{{
        { HeuristicType::Manhattan, "Manhattan" },
        { HeuristicType::Euclidean, "Euclidean" },
        { HeuristicType::Octile,    "Octile" },
        { HeuristicType::Chebyshev, "Chebyshev" }
    }};
    
    f32 CalculateHeuristicCost(HeuristicType type, f32 dx, f32 dy, f32 weight = 1.0f)
    {
        switch (type)
        {
            case HeuristicType::Manhattan:
                return HeuristicManhattan(dx, dy) * weight;

            case HeuristicType::Euclidean:
                return HeuristicEuclidean(dx, dy) * weight;

            case HeuristicType::Octile:
                return HeuristicOctile(dx, dy) * weight;

            case HeuristicType::Chebyshev:
                return HeuristicChebyshev(dx, dy) * weight;
        }

        // �⺻�� Octile
        return HeuristicOctile(dx, dy) * weight;
    }

    f32 CalculateHeuristicCost(HeuristicType type, f32 sx, f32 sy, f32 ex, f32 ey, f32 weight = 1.0f)
    {
        return CalculateHeuristicCost(type, sx - ex, ey - sy, weight);
    }
}

// Private Module Fragment : Optional
// Private Module Fragment�� �� ���(Primary Module) �ʿ����� ��� �����ϴ�.
// module: private;
