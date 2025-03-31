// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.Algorithm:Core;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    // sqrt(2)의 값을 매번 계산해서 사용하는 것을 방지하기 위한 값
    constexpr f32 kSqrt2 = 1.414213562373095f;

    // Grid 이동 비용
    constexpr f32 kGridCardinalCost = 1.0f;   // 동서남북(직선)
    constexpr f32 kGridDiagonalCost = kSqrt2; // 북동, 북서, 남동, 남서(대각선)
    
    // 탐색에 사용되는 모든 길찾기 알고리즘
    enum class AlgorithmType
    {
        DepthFirst,
        BreadthFirst,
        BestFirst, // Heuristic Search
        Dijkstra,
        AStar,
        JumpPoint,
        JumpPointPlus,

        Max,
    };

    struct AlgorithmTypeInfo
    {
        const AlgorithmType type{ };
        std::string_view name{ };
    };

    const std::array<AlgorithmTypeInfo, (i32)AlgorithmType::Max> kAlgorithmTypeInfoList{{
        { AlgorithmType::DepthFirst,    "Depth First Search" },
        { AlgorithmType::BreadthFirst,  "Breadth First Search" },
        { AlgorithmType::BestFirst,     "Best First Search" },
        { AlgorithmType::Dijkstra,      "Dijkstra" },
        { AlgorithmType::AStar,         "A*" },
        { AlgorithmType::JumpPoint,     "Jump Point Search" },
        { AlgorithmType::JumpPointPlus, "JPS+" },
    }};
    
    // Grid 계열 Node에서 인접 노드를 가리키는 방향
    enum class AdjacentDirection
    {
        /**
         *  d |     0     |   1   |     2     |
         * ------------------------------------
         *  0 | NorthWest | North | NorthEast |
         *  1 |    West   |  None |    East   |
         *  2 | SouthWest | South | SouthEast |
         *
         * dy, dx를 기반으로 캐스팅했을 때 위치를 바로 파악할 수 있어야 한다.
         * AdjacentDirection = (dx + 1) + ((dy + 1) * 3)
         */
        NorthWest,
        North,
        NorthEast,

        West,
        None,
        East,

        SouthWest,
        South,
        SouthEast,

        NumDirections, // 방향의 개수

        InvalidDirection,
    };

    // JPS에서 OpenNode가 나아갈 방향을 묶어서 보관 및 사용할 용도의 플래그 자료형
    // (주의) JPS+의 경우 8방향 Direction 별로 나아갈 수 있는 가능성을 배열 단위로 묶은 테이블을 사용하기 때문에 AdjacentDirection을 쓴다.
    enum class JpsDirectionFlags
    {
        /**
         * directionFlags = openNode.directionFlags;
         * 
         * for (i32 i = 0; i < NumFlags; i++)
         * {
         *     i32 dir = (1 << i);
         * 
         *     if (directionFlags & dir)
         *     {
         *         Jump(openNode.x, openNode.y, dir);
         *     }
         * }
         */
        None = 0,

        North = 1 << 0,
        East  = 1 << 1,
        South = 1 << 2,
        West  = 1 << 3,

        NorthWest = 1 << 4,
        NorthEast = 1 << 5,
        SouthEast = 1 << 6,
        SouthWest = 1 << 7,

        AllFlags = (1 << 8) - 1,

        NumFlags = 8,
    };

    // 경로를 계산하는 과정에서 사용되는 길찾기 노드
    struct PathFindingNode // struct PathNode
    {
        i32 x = -1;
        i32 y = -1;

        const PathFindingNode* parent = nullptr;

        // 방향 변수가 필요하게 되면 사용하도록 한다(디버깅 확인, JPS 계열, 그리는 작업 등).
        // 방향 값이 아닌 방향 플래그로 사용해야 할 수도 있다.
        // Dir enteringDirection; // indegree
        // Dir exitingDirection;  // outdegree

        // f(n) = g(n) + h(n)
        f32 f = 0.0f; // 최종 비용
        f32 g = 0.0f; // 이동 경로 비용
        f32 h = 0.0f; // 휴리스틱 예측 비용

        // JPS, JPS+
        union
        {
            i32 nextDirectionFlags = 0; // JPS에서 다음 노드가 진행할 방향을 플래그로 묶어서 저장
            AdjacentDirection indegreeDirection; // JPS+에서 이전 노드가 진행했던 방향을 토대로 다음에 진행할 방향 테이블을 찾기 위한 용도
        };

        /**
         * 노드를 set에서 찾는 과정을 생략하기 위해 노드 차원에서 set에 들어간 상태인지를 변수로 관리한다.
         * 특정 위치에서 특정 노드를 조회하면 바로 어떤 집합에 속해 있는지 알 수 있게 처리하는 것이 좋다.
         * 
         * 1. 방금 막 발견된 상황(isInOpenSet과 isInClosedSet가 false)
         * 2. OpenSet에 속해 있는 상황(isInOpenSet가 true)
         * 3. ClosedSet에 속해 있는 상황(isInClosedSet가 true)
         */ 
        bool isInOpenSet   = false;
        bool isInClosedSet = false;

        // 노드의 아이디가 아닌 길찾기 과정 자체에 대한 식별 번호이다.
        // 여기선 결과만 도출하는 방식의 길찾기에서 사용한다.
        ui32 pathFindingId = std::numeric_limits<ui32>::max();
    };

    struct PathFindingNodeComp
    {
        std::strong_ordering operator()(PathFindingNode* lhs, PathFindingNode* rhs)
        {
            // if (lhs->f < rhs->f)
            // {
            //     return std::strong_ordering::less;
            // }
            // else if (lhs->f > rhs->f)
            // {
            //     return std::strong_ordering::greater;
            // }
            // 
            // return std::strong_ordering::equal;
            
            // 비용이 적은 것일 수록 우선순위가 더 높다.
            return std::strong_order(lhs->f, rhs->f);
        }
    };

    /**
     * 길찾기 로직이 수행되었을 때 단계별 상태를 저장하기 위한 자료형이다.
     * 탐색 단위를 저장하기 때문에 커맨드 패턴 등과 연계해서 리플레이 기능 같은 걸 구현하면 된다.
     * 
     * 기본 방위만 탐색할 경우 예시
     * [ ][ ][ ]    [ ][ ][ ]    [D][ ][ ]
     * [ ][ ][ ] -> [B][ ][ ] -> [X][E][ ]
     * [A][ ][ ]    [X][C][ ]    [X][C][ ]
     *
     * First Record   | Second Record        | Third Record
     * - Visited : A  | - Visited : A, B, C  | - Visited : B, D, E
     * - Open : A     | - Open : B, C        | - Open : D, E
     * - Closed : -   | - Closed : A         | - Closed : B
     * 
     * 정보를 복원할 때 유의해야 하는 점
     * - 레코드 정보는 Stack이나 커맨드 패턴 등으로 관리하는 것이 좋음(복원 용이).
     * - OpenSet에 있는 노드는 삽입 삭제 외 갱신도 있음.\
     * - 탐색 정보를 읽으면 다음 단계로 넘어가는 것이고 반대로 정보를 제거하면 이전 단계로 되돌아가는 것이다.
     * 
     * 다음 상태 이동
     * 1. openNodes의 요소를 RecordInfoStack[y, x]에 Push한다(Open 상태도 첨부함).
     * 2. closedNodes의 요소를 RecordInfoStack[y, x]에 Push한다(Closed 상태도 첨부함).
     * 3. 다음 PathFindingRecord로 이동한다.
     * 
     * 이전 상태 복원
     * 1. 이전 PathFindingRecord로 이동한다.
     * 2. openNodes의 요소를 RecordInfoStack[y, x]에서 제거한다(Pop 진행).
     * 3. closedNodes의 요소를 RecordInfoStack[y, x]에서 제거한다(Pop 진행).
     */
    struct PathFindingRecord
    {
        enum class RecordType
        {
            None,

            Processing,  // 탐색의 기준이 되는 노드일 경우

            Visited,     // 탐색 과정에서 한 번이라도 고려되었을 경우
            OpenNew,     // 노드가 OpenSet에 막 추가된 상황
            OpenUpdated, // OpenSet에 있는 노드의 정보가 갱신된 상황
            Closed,      // 노드가 ClosedSet에 들어간 상황
        };

        struct RecordNode
        {
            RecordType recordType = RecordType::None;

            i32 x = -1;
            i32 y = -1;

            i32 parentX = -1;
            i32 parentY = -1;

            // f(n) = g(n) + h(n)
            f32 f = 0.0f; // 최종 비용
            f32 g = 0.0f; // 이동 경로 비용
            f32 h = 0.0f; // 휴리스틱 예측 비용
        };

        // 탐색의 기준이 되는 노드
        RecordNode processingNode;

        // 탐색 기준으로부터 원점까지의 경로
        std::vector<Vec2D<i32>> pathPoints;

        // 탐색된 모든 노드
        std::vector<RecordNode> visitedNodes;

        // OpenSet에 새로 삽입된 노드
        std::vector<RecordNode> openNewNodes;

        // OpenSet에서 갱신된 노드
        std::vector<RecordNode> openUpdatedNodes;

        // ClosedSet에 삽입된 노드
        std::vector<RecordNode> closedNodes;
        
        // PQ 수행 횟수
        i32 openSetEnqueuedCnt = 0;
        i32 openSetUpdatedCnt  = 0;
        i32 openSetDequeuedCnt = 0;

        /****************************************
        *      PathFindingRecord Functions      *
        ****************************************/
        void SetProcessingNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            processingNode.recordType = RecordType::Processing;

            processingNode.x = x;
            processingNode.y = y;

            processingNode.parentX = parentX;
            processingNode.parentY = parentY;

            processingNode.f = f;
            processingNode.g = g;
            processingNode.h = h;
        }

        template <typename T>
            requires std::same_as<T, std::vector<Vec2D<i32>>>
        void TakePathPoints(T&& pathPoints)
        {
            this->pathPoints = std::forward<T>(pathPoints);
        }

        void AddVisitedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            visitedNodes.push_back({ RecordType::Visited, x, y, parentX, parentY, f, g, h });
        }

        void AddOpenNewNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            openNewNodes.push_back({ RecordType::OpenNew, x, y, parentX, parentY, f, g, h });
        }

        void AddOpenUpdatedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            openUpdatedNodes.push_back({ RecordType::OpenUpdated, x, y, parentX, parentY, f, g, h });
        }

        void AddClosedNode(i32 x, i32 y, i32 parentX, i32 parentY, f32 f, f32 g, f32 h)
        {
            closedNodes.push_back({ RecordType::Closed, x, y, parentX, parentY, f, g, h });
        }

        void SetOpenSetInfo(i32 openSetEnqueuedCnt, i32 openSetUpdatedCnt, i32 openSetDequeuedCnt)
        {
            this->openSetEnqueuedCnt = openSetEnqueuedCnt;
            this->openSetUpdatedCnt  = openSetUpdatedCnt;
            this->openSetDequeuedCnt = openSetDequeuedCnt;
        }
    };

    // 부모 노드를 역추적하여 경로상의 이동 위치를 배열로 반환하는 함수 
    std::vector<Vec2D<i32>> RetracePath(const PathFindingNode& node)
    {
        const PathFindingNode* traceNode = &node;

        std::vector<Vec2D<i32>> pathNodes;

        while (traceNode != nullptr)
        {
            pathNodes.push_back({ traceNode->x, traceNode->y });

            traceNode = traceNode->parent;
        }

        std::ranges::reverse(pathNodes);

        return pathNodes;
    }

    // DeltaPos 값을 AdjacentDirection으로 바꿔주는 함수
    // dx : -1, 0, 1
    // dy : -1, 0, 1
    AdjacentDirection ConvertToNodeDirection(i32 dx, i32 dy)
    {
        i32 dir = (dx + 1) + ((dy + 1) * 3);

        if (dir < 0 || dir >= (i32)AdjacentDirection::NumDirections)
            return AdjacentDirection::InvalidDirection;

        return (AdjacentDirection)dir;
    }

    // AdjacentDirection을 { dx, dy }로 바꿔주는 함수
    Vec2D<i32> ConvertToDeltaPos(AdjacentDirection dir)
    {
        // static const i32 kDx[] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
        // static const i32 kDy[] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
        // 
        // return { kDx[(i32)dir], kDy[(i32)dir]};

        // 값을 0부터 순서대로 나열하면 컴파일러가 알아서 점프 테이블을 만들어서 최적화한다.
        switch (dir)
        {
            case AdjacentDirection::NorthWest:
                return { -1, -1 };

            case AdjacentDirection::North:
                return { 0, -1 };

            case AdjacentDirection::NorthEast:
                return { 1, -1 };

            case AdjacentDirection::West:
                return { -1, 0 };

            case AdjacentDirection::None:
                return { 0, 0 };

            case AdjacentDirection::East:
                return { 1, 0 };

            case AdjacentDirection::SouthWest:
                return { -1, 1 };

            case AdjacentDirection::South:
                return { 0, 1 };

            case AdjacentDirection::SouthEast:
                return { 1, 1 };

            default:
                return { 0, 0 };
        }

        return { 0, 0 };
    }

    // JPS+ : AdjacentDirection 방향으로 움직일 거리를 반환하는 함수
    Vec2D<i32> ConvertToDeltaPos(AdjacentDirection dir, i32 amount)
    {
        Vec2D<i32> deltaPos = ConvertToDeltaPos(dir);
        {
            deltaPos.x *= amount;
            deltaPos.y *= amount;
        }

        return deltaPos;
    }

    // AdjacentDirection의 크기를 반환하는 함수
    f32 ConvertToDistance(AdjacentDirection dir)
    {
        switch (dir)
        {
            case AdjacentDirection::NorthWest:
                return kGridDiagonalCost;

            case AdjacentDirection::North:
                return kGridCardinalCost;

            case AdjacentDirection::NorthEast:
                return kGridDiagonalCost;

            case AdjacentDirection::West:
                return kGridCardinalCost;

            case AdjacentDirection::None:
                return 0.0f;

            case AdjacentDirection::East:
                return kGridCardinalCost;

            case AdjacentDirection::SouthWest:
                return kGridDiagonalCost;

            case AdjacentDirection::South:
                return kGridCardinalCost;

            case AdjacentDirection::SouthEast:
                return kGridDiagonalCost;

            default:
                return 0.0f;
        }

        return 0.0f;
    }

    // DeltaPos 값을 JpsDirectionFlags로 바꿔주는 함수
    // dx : -1, 0, 1
    // dy : -1, 0, 1
    i32 ConvertToJpsDirectionFlags(i32 dx, i32 dy)
    {
        switch (dy)
        {
            // 북
            case -1:
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::NorthWest;
                    case  0: return (i32)JpsDirectionFlags::North;
                    case  1: return (i32)JpsDirectionFlags::NorthEast;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            // 수평(?)
            case 0: 
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::West;
                    case  1: return (i32)JpsDirectionFlags::East;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            // 남
            case 1:
            {
                switch (dx)
                {
                    case -1: return (i32)JpsDirectionFlags::SouthWest;
                    case  0: return (i32)JpsDirectionFlags::South;
                    case  1: return (i32)JpsDirectionFlags::SouthEast;

                    default:
                        return (i32)JpsDirectionFlags::None;
                }

                break;
            }

            default:
                return (i32)JpsDirectionFlags::None;
        }

        return 0;
    }

    // JPS+ : AdjacentDirection가 동서남북(상하좌우) 방향인지 확인하는 함수
    bool IsDirectionCardinal(AdjacentDirection adjDir)
    {
        switch (adjDir)
        {
            // case AdjacentDirection::North:
            // case AdjacentDirection::East:
            // case AdjacentDirection::South:
            // case AdjacentDirection::West:
            //     return true;

            // 점프 테이블 최적화 유도
            case AdjacentDirection::NorthWest:
                return false;

            case AdjacentDirection::North:
                return true;

            case AdjacentDirection::NorthEast:
                return false;

            case AdjacentDirection::West:
                return true;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return true;

            case AdjacentDirection::SouthWest:
                return false;

            case AdjacentDirection::South:
                return true;

            case AdjacentDirection::SouthEast:
                return false;
        }

        return false;
    }

    // JPS+ : AdjacentDirection가 대각선 방향인지 확인하는 함수
    bool IsDirectionDiagonal(AdjacentDirection adjDir)
    {
        switch (adjDir)
        {
            // case AdjacentDirection::kNorthWest:
            // case AdjacentDirection::kNorthEast:
            // case AdjacentDirection::kSouthEast:
            // case AdjacentDirection::kSouthWest:
            //     return true;

           // 점프 테이블 최적화 유도
            case AdjacentDirection::NorthWest:
                return true;

            case AdjacentDirection::North:
                return false;

            case AdjacentDirection::NorthEast:
                return true;

            case AdjacentDirection::West:
                return false;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return false;

            case AdjacentDirection::SouthWest:
                return true;

            case AdjacentDirection::South:
                return false;

            case AdjacentDirection::SouthEast:
                return true;
        }

        return false;
    }
     
    // JPS+ : 목표가 정확한 방향에 속해 있는지 확인(동서남북(상하좌우)만 체크)
    // 만약 대각선 방향으로 정확한 방향을 확인하고 싶다면 (abs(diffX) == abs(diffY))를 쓸 것!
    bool IsDestinationInExactDirection(Vec2D<i32> here, Vec2D<i32> destPos, AdjacentDirection adjDir)
    {
        i32 diffX = destPos.x - here.x;
        i32 diffY = destPos.y - here.y;

        // !! 위로 갈수록 감소, 아래로 갈수록 증가 !!
        switch (adjDir)
        {
            // case AdjacentDirection::North:
            //     return (diffX == 0) && (diffY < 0);
            // 
            // case AdjacentDirection::East:
            //     return (diffX > 0) && (diffY == 0);
            // 
            // case AdjacentDirection::South:
            //     return (diffX == 0) && (diffY > 0);
            // 
            // case AdjacentDirection::West:
            //     return (diffX < 0) && (diffY == 0);

            // 점프 테이블 최적화 유도
            case AdjacentDirection::NorthWest:
                return false;

            case AdjacentDirection::North:
                return (diffX == 0) && (diffY < 0);

            case AdjacentDirection::NorthEast:
                return false;

            case AdjacentDirection::West:
                return (diffX < 0) && (diffY == 0);

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return (diffX > 0) && (diffY == 0);

            case AdjacentDirection::SouthWest:
                return false;

            case AdjacentDirection::South:
                return (diffX == 0) && (diffY > 0);

            case AdjacentDirection::SouthEast:
                return false;
        }

        return false;
    }

    // JPS+ : 목표가 사분면 방향에 속해 있는지 확인(대각선만 체크)
    // 직선 상에 위치한 것을 체크하고 싶다면 IsDestinationInExactDirection()의 상하좌우 코드를 옮기도록 하자.
    bool IsDestinationInGeneralDirection(Vec2D<i32> here, Vec2D<i32> destPos, AdjacentDirection adjDir)
    {
        i32 diffX = destPos.x - here.x;
        i32 diffY = destPos.y - here.y;
    
        // !! 위로 갈수록 감소, 아래로 갈수록 증가 !!
        switch (adjDir)
        {
            // case AdjacentDirection::NorthWest:
            //     return (diffX < 0) && (diffY < 0);
            // 
            // case AdjacentDirection::NorthEast:
            //     return (diffX > 0) && (diffY < 0);
            // 
            // case AdjacentDirection::SouthEast:
            //     return (diffX > 0) && (diffY > 0);
            // 
            // case AdjacentDirection::SouthWest:
            //     return (diffX < 0) && (diffY > 0);

            // 점프 테이블 최적화 유도
            case AdjacentDirection::NorthWest:
                return (diffX < 0) && (diffY < 0);

            case AdjacentDirection::North:
                return false;

            case AdjacentDirection::NorthEast:
                return (diffX > 0) && (diffY < 0);

            case AdjacentDirection::West:
                return false;

            case AdjacentDirection::None:
                return false;

            case AdjacentDirection::East:
                return false;

            case AdjacentDirection::SouthWest:
                return (diffX < 0) && (diffY > 0);

            case AdjacentDirection::South:
                return false;

            case AdjacentDirection::SouthEast:
                return (diffX > 0) && (diffY > 0);
        }
    
        return false;
    }
}

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
