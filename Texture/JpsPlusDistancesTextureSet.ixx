// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
export module PathFinding.JpsPlusDistancesTextureSet;

import CoroutineTask;

import Engine2D.Graphics;

import PathFinding.Presentation;

// Module Purview / Module Interface : Optional
export namespace PathFinding
{
    class StaticMap;

    class JpsPlusDistancesTextureSet final : public std::enable_shared_from_this<JpsPlusDistancesTextureSet>
    {
    private:
        struct Deleter
        {
            void operator()(JpsPlusDistancesTextureSet* instance) const;
        };

    private:
        /*************************
        *      Rule of Five      *
        *************************/
        explicit JpsPlusDistancesTextureSet() = default; // constructor
        ~JpsPlusDistancesTextureSet() = default; // destructor

        JpsPlusDistancesTextureSet(const JpsPlusDistancesTextureSet& rhs) = delete; // copy constructor
        JpsPlusDistancesTextureSet& operator=(const JpsPlusDistancesTextureSet& rhs) = delete; // copy assignment

        JpsPlusDistancesTextureSet(JpsPlusDistancesTextureSet&& rhs) noexcept = delete; // move constructor
        JpsPlusDistancesTextureSet& operator=(JpsPlusDistancesTextureSet&& rhs) noexcept = delete; // move assignment

    public:
        static std::shared_ptr<JpsPlusDistancesTextureSet> Create(Renderer& renderer);

    public:
        bool CoroStartLoadingTextures(std::shared_ptr<StaticMap>& staticMap);
        void CoroStopLoadingTextures();

        void CoroProgress();

    public:
        bool IsDone() { return _texturesTask.Done(); }

        std::shared_ptr<RenderTargetTexture> GetPrimaryJpTexture(CellSizeType type)
        {
            return _primaryJpTextures[(i32)type];
        }

        std::shared_ptr<RenderTargetTexture> GetStraightJpTexture(CellSizeType type)
        {
            return _straightJpTextures[(i32)type];
        }

        std::shared_ptr<RenderTargetTexture> GetDiagonalJpTexture(CellSizeType type)
        {
            return _diagonalJpTextures[(i32)type];
        }

        std::shared_ptr<RenderTargetTexture> GetWallDiatancesTexture(CellSizeType type)
        {
            return _wallDistancesTextures[(i32)type];
        }

    private:
        CoroutineTask coroLoadTextures(std::shared_ptr<StaticMap> staticMap);

    private:
        Renderer* _renderer = nullptr;

    private:
        std::array<NumberTextureSet*, (i32)CellSizeType::Max> _jpDistTexSets;
        std::array<NumberTextureSet*, (i32)CellSizeType::Max> _wallDistTexSets;

        std::array<std::shared_ptr<RenderTargetTexture>, (i32)CellSizeType::Max> _primaryJpTextures;
        std::array<std::shared_ptr<RenderTargetTexture>, (i32)CellSizeType::Max> _straightJpTextures;
        std::array<std::shared_ptr<RenderTargetTexture>, (i32)CellSizeType::Max> _diagonalJpTextures;
        std::array<std::shared_ptr<RenderTargetTexture>, (i32)CellSizeType::Max> _wallDistancesTextures;

    private:
        CoroutineTask _texturesTask;
    };
}

// Private Module Fragment : Optional
module: private;