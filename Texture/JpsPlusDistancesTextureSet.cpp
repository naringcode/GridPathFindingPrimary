// Global Module Fragment : Optional
module;

#include "Engine2D/Headers.h"

// Module Preamble : Required
module PathFinding.JpsPlusDistancesTextureSet;

import Stopwatch;

import PathFinding.Map;

// Module Purview / Module Interface : Optional
BEGIN_NS(PathFinding)

void JpsPlusDistancesTextureSet::Deleter::operator()(JpsPlusDistancesTextureSet* instance) const
{
    if (instance == nullptr)
        return;

    instance->_renderer = nullptr;

    std::ranges::for_each(instance->_jpDistTexSets, NumberTextureSet::Destroy);
    std::ranges::for_each(instance->_wallDistTexSets, NumberTextureSet::Destroy);

    delete instance;
}

std::shared_ptr<JpsPlusDistancesTextureSet> JpsPlusDistancesTextureSet::Create(Renderer& renderer)
{
    JpsPlusDistancesTextureSet* instance = new JpsPlusDistancesTextureSet{ };

    instance->_renderer = &renderer;

    for (i32 idx : std::views::iota(0, (i32)CellSizeType::Max))
    {
        i32 ptSize = std::max((i32)((kCellSizeTypeInfoList[idx].size / 64.0f) * 12), 3);

        // Windows 내장 폰트 사용하기(운영체제가 C 드라이브에 설치되어 있어야 함)
        instance->_jpDistTexSets[idx]   = NumberTextureSet::Create("C:\\Windows\\Fonts\\Arial.ttf", ptSize, { 13, 13, 213, 255 });
        instance->_wallDistTexSets[idx] = NumberTextureSet::Create("C:\\Windows\\Fonts\\Arial.ttf", ptSize, { 255, 25, 25, 255 });

        instance->_jpDistTexSets[idx]->SetBlendMode(NumberTextureSet::BlendMode::None);
        instance->_wallDistTexSets[idx]->SetBlendMode(NumberTextureSet::BlendMode::None);
    }

    return std::shared_ptr<JpsPlusDistancesTextureSet>{ instance, Deleter{ } };
}

bool JpsPlusDistancesTextureSet::CoroStartLoadingTextures(std::shared_ptr<StaticMap>& staticMap)
{
    CoroStopLoadingTextures();

    if (staticMap->IsOptimizedForJpsPlus() == false)
        return false;

    // std::zip_view를 쓰면 편한데 C++23부터 이건 지원한다.
    for (i32 idx : std::views::iota(0, std::ssize(kCellSizeTypeInfoList)))
    {
        Vec2D<i32> spanSize;
        spanSize.x = staticMap->GetWidth()  * (kCellSizeTypeInfoList[idx].size + kCellSpacing) + kCellSpacing;
        spanSize.y = staticMap->GetHeight() * (kCellSizeTypeInfoList[idx].size + kCellSpacing) + kCellSpacing;

        RenderTargetTexture* primaryJpTex     = RenderTargetTexture::Create(spanSize.x, spanSize.y);
        RenderTargetTexture* straightJpTex    = RenderTargetTexture::Create(spanSize.x, spanSize.y);
        RenderTargetTexture* diagonalJpTex    = RenderTargetTexture::Create(spanSize.x, spanSize.y);
        RenderTargetTexture* wallDistancesTex = RenderTargetTexture::Create(spanSize.x, spanSize.y);

        primaryJpTex->BeginRenderTarget();
        {
            primaryJpTex->Clear(255, 255, 255, 0);
        }
        primaryJpTex->EndRenderTarget();

        straightJpTex->BeginRenderTarget();
        {
            straightJpTex->Clear(255, 255, 255, 0);
        }
        straightJpTex->EndRenderTarget();

        diagonalJpTex->BeginRenderTarget();
        {
            diagonalJpTex->Clear(255, 255, 255, 0);
        }
        diagonalJpTex->EndRenderTarget();

        wallDistancesTex->BeginRenderTarget();
        {
            wallDistancesTex->Clear(255, 255, 255, 0);
        }
        wallDistancesTex->EndRenderTarget();

        _primaryJpTextures[idx]  = std::shared_ptr<RenderTargetTexture>{ primaryJpTex, RenderTargetTexture::Destroy };
        _straightJpTextures[idx] = std::shared_ptr<RenderTargetTexture>{ straightJpTex, RenderTargetTexture::Destroy };
        _diagonalJpTextures[idx] = std::shared_ptr<RenderTargetTexture>{ diagonalJpTex, RenderTargetTexture::Destroy };
        _wallDistancesTextures[idx] = std::shared_ptr<RenderTargetTexture>{ wallDistancesTex, RenderTargetTexture::Destroy };
    }

    _texturesTask = coroLoadTextures(staticMap);
    _texturesTask.Resume();

    return true;
}

void JpsPlusDistancesTextureSet::CoroStopLoadingTextures()
{
    _texturesTask.Destroy();
}

void JpsPlusDistancesTextureSet::CoroProgress()
{
    if (_texturesTask.Done() == true)
        return;

    _texturesTask.Resume();
}

CoroutineTask JpsPlusDistancesTextureSet::coroLoadTextures(std::shared_ptr<StaticMap> staticMap)
{
    std::shared_ptr<JpsPlusDistancesTextureSet> self = this->shared_from_this();

    Stopwatch stopwatch;
    f64       elapsed = 0.0f;

    co_await std::suspend_always{ };

    stopwatch.Start();

    /********************************
    *      Primary Jump Points      *
    ********************************/
    for (i32 idx : std::views::iota(0, staticMap->GetSize()))
    {
        stopwatch.CaptureLapTime();
        elapsed += stopwatch.GetLapTimeSeconds();

        // 20ms마다 로딩 지연
        if (elapsed >= 0.02)
        {
            co_await std::suspend_always{ };

            stopwatch.Start();
            elapsed = 0.0;
        }

        i32 x = idx % staticMap->GetWidth();
        i32 y = idx / staticMap->GetWidth();

        if (staticMap->IsWalkableNodeAt(x, y) == false)
            continue;

        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(x, y);

        if (jumpPointNode.primaryIndegreeFlags == (i32)JpsDirectionFlags::None)
            continue;

        std::ranges::for_each(kCellSizeTypeInfoList, [&, idx = 0](const CellSizeTypeInfo& item) mutable {

            i32 leftX = x * (item.size + kCellSpacing) + kCellSpacing + 1;
            i32 topY  = y * (item.size + kCellSpacing) + kCellSpacing + 1;

            i32 rightX  = leftX + item.size - 3;
            i32 bottomY = topY  + item.size - 3;

            i32 centerX = (leftX + rightX) / 2 + 1;
            i32 centerY = (topY + bottomY) / 2 + 1;

            // Circle Size(Primary Jump Point)
            i32 radius = std::max((i32)((item.size / 16.0f) * 2.8f), 2);
            
            // Arrow Size(Indirection)
            i32 arrowSize = (i32)((item.size / 16.0f) * 3.0f);
            i32 offPos    = (i32)((item.size / 16.0f));

            // Primary Jump Points
            _primaryJpTextures[idx]->BeginRenderTarget();
            {
                _renderer->DrawFilledCircle(centerX, centerY, radius, { 55, 55, 255 });
                
                if ((i32)JpsDirectionFlags::North & jumpPointNode.primaryIndegreeFlags)
                {
                    // _renderer->DrawThickLine(centerX, centerY + offPos + 1, centerX, bottomY, thick, { 0, 0, 0 });

                    _renderer->DrawFilledTriangle(centerX, centerY + offPos,
                                                  centerX - arrowSize / 2, centerY + offPos + arrowSize,
                                                  centerX + arrowSize / 2, centerY + offPos + arrowSize, { 0, 0, 0 });
                }

                if ((i32)JpsDirectionFlags::East & jumpPointNode.primaryIndegreeFlags)
                {
                    // _renderer->DrawThickLine(centerX - offPos - 1, centerY, leftX, centerY, thick, { 0, 0, 0 });

                    _renderer->DrawFilledTriangle(centerX - offPos, centerY,
                                                  centerX - offPos - arrowSize, centerY - arrowSize / 2,
                                                  centerX - offPos - arrowSize, centerY + arrowSize / 2, { 0, 0, 0 });
                }

                if ((i32)JpsDirectionFlags::South & jumpPointNode.primaryIndegreeFlags)
                {
                    // _renderer->DrawThickLine(centerX, centerY - offPos - 1, centerX, topY, thick, { 0, 0, 0 });

                    _renderer->DrawFilledTriangle(centerX, centerY - offPos,
                                                  centerX - arrowSize / 2, centerY - offPos - arrowSize,
                                                  centerX + arrowSize / 2, centerY - offPos - arrowSize, { 0, 0, 0 });
                }

                if ((i32)JpsDirectionFlags::West & jumpPointNode.primaryIndegreeFlags)
                {
                    // _renderer->DrawThickLine(centerX + offPos + 1, centerY, rightX, centerY, thick, { 0, 0, 0 });

                    _renderer->DrawFilledTriangle(centerX + offPos, centerY,
                                                  centerX + offPos + arrowSize, centerY - arrowSize / 2,
                                                  centerX + offPos + arrowSize, centerY + arrowSize / 2, { 0, 0, 0 });
                }
            }
            _primaryJpTextures[idx]->EndRenderTarget();

            idx++;
        });
    }

    /***********************************
    *      Render Number Textures      *
    ***********************************/
    for (i32 idx : std::views::iota(0, staticMap->GetSize()))
    {
        stopwatch.CaptureLapTime();
        elapsed += stopwatch.GetLapTimeSeconds();

        // 20ms마다 로딩 지연
        if (elapsed >= 0.02)
        {
            co_await std::suspend_always{ };

            stopwatch.Start();
            elapsed = 0.0;
        }

        i32 x = idx % staticMap->GetWidth();
        i32 y = idx / staticMap->GetWidth();

        if (staticMap->IsWalkableNodeAt(x, y) == false)
            continue;

        const StaticMap::JumpPointNode& jumpPointNode = staticMap->GetJumpPointNodeAt(x, y);

        std::ranges::for_each(kCellSizeTypeInfoList, [&, idx = 0](const CellSizeTypeInfo& item) mutable {

            i32 drawX = x * (item.size + kCellSpacing) + kCellSpacing;
            i32 drawY = y * (item.size + kCellSpacing) + kCellSpacing;

            i32 drawSize = item.size;

            i32 offPos = std::max((i32)((item.size / 64.0f) * 3.0f), 1);

            drawX += offPos;
            drawY += offPos;

            drawSize -= offPos * 2;

            // Straight Jump Points
            _straightJpTextures[idx]->BeginRenderTarget();
            {
                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::North] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::North], drawX, drawY, drawSize, drawSize, RenderPivot::CenterTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::East] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::East], drawX, drawY, drawSize, drawSize, RenderPivot::RightCenter);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::South] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::South], drawX, drawY, drawSize, drawSize, RenderPivot::CenterBottom);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::West] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::West], drawX, drawY, drawSize, drawSize, RenderPivot::LeftCenter);
                }
            }
            _straightJpTextures[idx]->EndRenderTarget();

            // Diagonal Jump Points
            _diagonalJpTextures[idx]->BeginRenderTarget();
            {
                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthWest] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthWest], drawX, drawY, drawSize, drawSize, RenderPivot::LeftTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthEast] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthEast], drawX, drawY, drawSize, drawSize, RenderPivot::RightTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthEast] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthEast], drawX, drawY, drawSize, drawSize, RenderPivot::RightBottom);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthWest] > 0)
                {
                    _renderer->DrawNumber(*_jpDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthWest], drawX, drawY, drawSize, drawSize, RenderPivot::LeftBottom);
                }
            }
            _diagonalJpTextures[idx]->EndRenderTarget();

            // Wall Distances
            _wallDistancesTextures[idx]->BeginRenderTarget();
            {
                // Straight
                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::North] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::North], drawX, drawY, drawSize, drawSize, RenderPivot::CenterTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::East] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::East], drawX, drawY, drawSize, drawSize, RenderPivot::RightCenter);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::South] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::South], drawX, drawY, drawSize, drawSize, RenderPivot::CenterBottom);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::West] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::West], drawX, drawY, drawSize, drawSize, RenderPivot::LeftCenter);
                }

                // Diagonal
                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthWest] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthWest], drawX, drawY, drawSize, drawSize, RenderPivot::LeftTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthEast] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::NorthEast], drawX, drawY, drawSize, drawSize, RenderPivot::RightTop);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthEast] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthEast], drawX, drawY, drawSize, drawSize, RenderPivot::RightBottom);
                }

                if (jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthWest] <= 0)
                {
                    _renderer->DrawNumber(*_wallDistTexSets[idx], jumpPointNode.jumpDistanceTable[(i32)AdjacentDirection::SouthWest], drawX, drawY, drawSize, drawSize, RenderPivot::LeftBottom);
                }
            }
            _wallDistancesTextures[idx]->EndRenderTarget();

            idx++;
        });
    }

    co_return;
}

END_NS

// Private Module Fragment : Optional
// Private Module Fragment는 주 모듈(Primary Module) 쪽에서만 사용 가능하다.
// module: private;
