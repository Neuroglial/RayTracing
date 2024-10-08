#ifndef ARFILM_H
#define ARFILM_H

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Utils/Color.h"
#include "Render/Filter.h"
#include "Utils/Parallel.h"

#include <memory>
#include <vector>

namespace RT
{

	class Film final : public Object
	{
	public:
		typedef std::shared_ptr<Film> ptr;

		Film(const PropertyTreeNode &node);
		Film(const Vec2i &resolution, const BBox2f &cropWindow,
			std::unique_ptr<Filter> filter, const std::string &filename, Float diagonal = 35.f,
			Float scale = 1.f, Float maxSampleLuminance = Infinity);

		BBox2i getSampleBounds() const;
		const Vec2i getResolution() const { return m_resolution; }

		std::unique_ptr<FilmTile> getFilmTile(const BBox2i &sampleBounds);
		void mergeFilmTile(std::unique_ptr<FilmTile> tile);

		void writeImageToFile(Float splatScale = 1);

		void setImage(const Spectrum *img) const;
		void addSplat(const Vec2f &p, Spectrum v);

		void clear();

		virtual void activate() override { initialize(); }

		virtual ClassType getClassType() const override { return ClassType::AEFilm; }
		virtual std::string toString() const override { return "Film[]"; }

	private:
		void initialize();

	private:

		//Note: XYZ is a display independent representation of color,
		//      and this is why we choose to use XYZ color herein.
		struct Pixel 
		{
			Pixel() 
			{ 
				m_xyz[0] = m_xyz[1] = m_xyz[2] = m_filterWeightSum = 0; 
			}

			Float m_xyz[3];				//xyz color of the pixel
			Float m_filterWeightSum;	//the sum of filter weight values
			AAtomicFloat m_splatXYZ[3]; //unweighted sum of samples splats
			Float m_pad;				//unused, ensure sizeof(APixel) -> 32 bytes
		};

		Vec2i m_resolution; //(width, height)
		std::string m_filename;
		std::unique_ptr<Pixel[]> m_pixels;

		Float m_diagonal;
		BBox2i m_croppedPixelBounds;	//actual rendering window

		std::unique_ptr<Filter> m_filter;
		std::mutex m_mutex;

		//Note: precomputed filter weights table
		static constexpr int filterTableWidth = 16;
		Float m_filterTable[filterTableWidth * filterTableWidth];

		Float m_scale;
		Float m_maxSampleLuminance;

		Pixel &getPixel(const Vec2i &p)
		{
			CHECK(insideExclusive(p, m_croppedPixelBounds));
			int width = m_croppedPixelBounds.m_pMax.x - m_croppedPixelBounds.m_pMin.x;
			int index = (p.x - m_croppedPixelBounds.m_pMin.x) + (p.y - m_croppedPixelBounds.m_pMin.y) * width;
			return m_pixels[index];
		}

	};

	struct FilmTilePixel
	{
		Spectrum m_contribSum = 0.f;		//sum of the weighted spectrum contributions
		Float m_filterWeightSum = 0.f;		//sum of the filter weights
	};

	class FilmTile final
	{
	public:
		// FilmTile Public Methods
		FilmTile(const BBox2i &pixelBounds, const Vec2f &filterRadius, const Float *filterTable,
			int filterTableSize, Float maxSampleLuminance)
			: m_pixelBounds(pixelBounds), m_filterRadius(filterRadius),
			m_invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
			m_filterTable(filterTable), m_filterTableSize(filterTableSize),
			m_maxSampleLuminance(maxSampleLuminance) 
		{
			m_pixels = std::vector<FilmTilePixel>(glm::max(0, pixelBounds.area()));
		}

		void addSample(const Vec2f &pFilm, Spectrum L, Float sampleWeight = 1.f) 
		{
			//限制最大亮度
			if (L.luminance() > m_maxSampleLuminance)
				L *= m_maxSampleLuminance / L.luminance();

			// 计算样本的光栅边界，p0，p1为当前过滤器下该采样点影响的范围
			Vec2f pFilmDiscrete = pFilm - Vec2f(0.5f, 0.5f);
			Vec2i p0 = (Vec2i)ceil(pFilmDiscrete - m_filterRadius);
			Vec2i p1 = (Vec2i)floor(pFilmDiscrete + m_filterRadius) + Vec2i(1, 1);
			p0 = max(p0, m_pixelBounds.m_pMin);
			p1 = min(p1, m_pixelBounds.m_pMax);

			// 循环过滤器并向像素阵列添加样本

			// 预计算在过滤表中的偏移量
			int *ifx = ALLOCA(int, p1.x - p0.x);
			for (int x = p0.x; x < p1.x; ++x) 
			{
				Float fx = glm::abs((x - pFilmDiscrete.x) * m_invFilterRadius.x * m_filterTableSize);
				ifx[x - p0.x] = glm::min((int)glm::floor(fx), m_filterTableSize - 1);
			}

			int *ify = ALLOCA(int, p1.y - p0.y);
			for (int y = p0.y; y < p1.y; ++y) 
			{
				Float fy = std::abs((y - pFilmDiscrete.y) * m_invFilterRadius.y * m_filterTableSize);
				ify[y - p0.y] = glm::min((int)std::floor(fy), m_filterTableSize - 1);
			}

			for (int y = p0.y; y < p1.y; ++y) 
			{
				for (int x = p0.x; x < p1.x; ++x) 
				{
					// 在（x，y）像素处计算滤波器值
					int offset = ify[y - p0.y] * m_filterTableSize + ifx[x - p0.x];
					Float filterWeight = m_filterTable[offset];

					// 使用过滤后的样本贡献更新像素值
					FilmTilePixel &pixel = getPixel(Vec2i(x, y));
					pixel.m_contribSum += L * sampleWeight * filterWeight;
					pixel.m_filterWeightSum += filterWeight;
				}
			}
		}

		FilmTilePixel &getPixel(const Vec2i &p) 
		{
			CHECK(insideExclusive(p, m_pixelBounds));
			int width = m_pixelBounds.m_pMax.x - m_pixelBounds.m_pMin.x;
			int index = (p.x - m_pixelBounds.m_pMin.x) + (p.y - m_pixelBounds.m_pMin.y) * width;
			return m_pixels[index];
		}

		const FilmTilePixel &getPixel(const Vec2i &p) const 
		{
			CHECK(insideExclusive(p, m_pixelBounds));
			int width =m_pixelBounds.m_pMax.x - m_pixelBounds.m_pMin.x;
			int index = (p.x - m_pixelBounds.m_pMin.x) + (p.y - m_pixelBounds.m_pMin.y) * width;
			return m_pixels[index];
		}

		BBox2i getPixelBounds() const { return m_pixelBounds; }

	private:
		const BBox2i m_pixelBounds;
		const Vec2f m_filterRadius, m_invFilterRadius;
		const Float *m_filterTable;
		const int m_filterTableSize;
		std::vector<FilmTilePixel> m_pixels;
		const Float m_maxSampleLuminance;
		
		friend class Film;
	};

}

#endif