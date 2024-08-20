#include "Object/Film.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

namespace RT
{
	AURORA_REGISTER_CLASS(Film, "Film")

	Film::Film(const PropertyTreeNode &node)
	{
		const auto &props = node.getPropertyList();
		Vec2f _res = props.getVector2f("Resolution", Vec2f(800, 600));
		m_resolution = Vec2i(static_cast<int>(_res.x), static_cast<int>(_res.y));
		m_filename = props.getString("Filename", "rendered.png");

		Vec2f _cropMin = props.getVector2f("CropMin", Vec2f(0.0f));
		Vec2f _cropMax = props.getVector2f("CropMax", Vec2f(1.0f));
		//Compute film image bounds
		//Note: cropWindow range [0,1]x[0,1]
		m_croppedPixelBounds =
			BBox2i(
				Vec2i(glm::ceil(m_resolution.x * _cropMin.x), glm::ceil(m_resolution.y * _cropMin.y)),
				Vec2i(glm::ceil(m_resolution.x * _cropMax.x), glm::ceil(m_resolution.y * _cropMax.y)));
		LOG(INFO) << "Created film with full resolution " << m_resolution <<
			". Crop window -> croppedPixelBounds " << m_croppedPixelBounds;

		m_diagonal = props.getFloat("Diagonal", 35.f);
		m_scale = props.getFloat("Scale", 1.0f);
		m_maxSampleLuminance = props.getFloat("MaxLum", Infinity);

		//Filter
		{
			const auto &filterNode = node.getPropertyChild("Filter");
			m_filter = std::unique_ptr<Filter>(static_cast<Filter*>(ObjectFactory::createInstance(
				filterNode.getTypeName(), filterNode)));
		}

		activate();
	}

	Film::Film(const Vec2i &resolution, const BBox2f &cropWindow, std::unique_ptr<Filter> filter,
		const std::string &filename, Float diagonal, Float scale, Float maxSampleLuminance)
		: m_resolution(resolution), m_filter(std::move(filter)), m_diagonal(diagonal),
		 m_filename(filename), m_scale(scale), m_maxSampleLuminance(maxSampleLuminance)
	{
		//Compute film image bounds
		//Note: cropWindow range [0,1]x[0,1]
		m_croppedPixelBounds =
			BBox2i(
				Vec2i(glm::ceil(m_resolution.x * cropWindow.m_pMin.x),
					glm::ceil(m_resolution.y * cropWindow.m_pMin.y)),
				Vec2i(glm::ceil(m_resolution.x * cropWindow.m_pMax.x),
					glm::ceil(m_resolution.y * cropWindow.m_pMax.y)));
		LOG(INFO) << "Created film with full resolution " << resolution <<
			". Crop window of " << cropWindow << " -> croppedPixelBounds " << m_croppedPixelBounds;

		initialize();
	}

	void Film::initialize()
	{
		m_pixels = std::unique_ptr<Pixel[]>(new Pixel[m_croppedPixelBounds.area()]);

		//预计算过滤器重量表注意：我们假设滤波函数f（x，y） = f（ | x | ， | y | ），因此只存储滤波器偏移正象限的值
		int offset = 0;
		for (int y = 0; y < filterTableWidth; ++y)
		{
			for (int x = 0; x < filterTableWidth; ++x, ++offset)
			{
				Vec2f p;
				p.x = (x + 0.5f) * m_filter->m_radius.x / filterTableWidth;
				p.y = (y + 0.5f) * m_filter->m_radius.y / filterTableWidth;
				m_filterTable[offset] = m_filter->evaluate(p);
			}
		}
	}

	BBox2i Film::getSampleBounds() const
	{
		BBox2f floatBounds(
			floor(Vec2f(m_croppedPixelBounds.m_pMin) + Vec2f(0.5f, 0.5f) - m_filter->m_radius),
			ceil(Vec2f(m_croppedPixelBounds.m_pMax) - Vec2f(0.5f, 0.5f) + m_filter->m_radius));
		return (BBox2i)floatBounds;
	}

	std::unique_ptr<FilmTile> Film::getFilmTile(const BBox2i &sampleBounds)
	{
		Vec2f halfPixel = Vec2f(0.5f, 0.5f);
		BBox2f floatBounds = (BBox2f)sampleBounds;
		Vec2i p0 = (Vec2i)ceil(floatBounds.m_pMin - halfPixel - m_filter->m_radius);
		Vec2i p1 = (Vec2i)floor(floatBounds.m_pMax - halfPixel + m_filter->m_radius) + Vec2i(1, 1);
		BBox2i tilePixelBounds = intersect(BBox2i(p0, p1), m_croppedPixelBounds);
		return std::unique_ptr<FilmTile>(new FilmTile(tilePixelBounds, m_filter->m_radius,
			m_filterTable, filterTableWidth, m_maxSampleLuminance));
	}

	void Film::mergeFilmTile(std::unique_ptr<FilmTile> tile)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (Vec2i pixel : tile->getPixelBounds()) 
		{
			// Merge _pixel_ into _Film::pixels_
			const FilmTilePixel &tilePixel = tile->getPixel(pixel);
			Pixel &mergePixel = getPixel(pixel);
			Float xyz[3];
			tilePixel.m_contribSum.toXYZ(xyz);
			for (int i = 0; i < 3; ++i)
			{
				mergePixel.m_xyz[i] += xyz[i];
			}
			mergePixel.m_filterWeightSum += tilePixel.m_filterWeightSum;
		}
	}

	void Film::writeImageToFile(Float splatScale)
	{
		LOG(INFO) << "Converting image to RGB and computing final weighted pixel values";
		std::unique_ptr<Float[]> rgb(new Float[3 * m_croppedPixelBounds.area()]);
		std::unique_ptr<Byte[]>  dst(new Byte[3 * m_croppedPixelBounds.area()]);
		int offset = 0;
		for (Vec2i p : m_croppedPixelBounds) 
		{
			// Convert pixel XYZ color to RGB
			Pixel &pixel = getPixel(p);
			XYZToRGB(pixel.m_xyz, &rgb[3 * offset]);

			// Normalize pixel with weight sum
			Float filterWeightSum = pixel.m_filterWeightSum;
			if (filterWeightSum != 0) 
			{
				Float invWt = (Float)1 / filterWeightSum;
				rgb[3 * offset + 0] = glm::max((Float)0, rgb[3 * offset + 0] * invWt);
				rgb[3 * offset + 1] = glm::max((Float)0, rgb[3 * offset + 1] * invWt);
				rgb[3 * offset + 2] = glm::max((Float)0, rgb[3 * offset + 2] * invWt);
			}

			// Add splat value at pixel
			Float splatRGB[3];
			Float splatXYZ[3] = { pixel.m_splatXYZ[0], pixel.m_splatXYZ[1],  pixel.m_splatXYZ[2] };
			XYZToRGB(splatXYZ, splatRGB);
			rgb[3 * offset + 0] += splatScale * splatRGB[0];
			rgb[3 * offset + 1] += splatScale * splatRGB[1];
			rgb[3 * offset + 2] += splatScale * splatRGB[2];

			// Scale pixel value by _scale_
			rgb[3 * offset + 0] *= m_scale;
			rgb[3 * offset + 1] *= m_scale;
			rgb[3 * offset + 2] *= m_scale;

#define TO_BYTE(v) (uint8_t) clamp(255.f * gammaCorrect(v) + 0.5f, 0.f, 255.f)
			dst[3 * offset + 0] = TO_BYTE(rgb[3 * offset + 0]);
			dst[3 * offset + 1] = TO_BYTE(rgb[3 * offset + 1]);
			dst[3 * offset + 2] = TO_BYTE(rgb[3 * offset + 2]);

			++offset;
		}

		LOG(INFO) << "Writing image " << m_filename << " with bounds " << m_croppedPixelBounds;
		auto extent = m_croppedPixelBounds.diagonal();
		stbi_write_png(m_filename.c_str(),
			extent.x,
			extent.y,
			3,
			static_cast<void*>(dst.get()),
			extent.x * 3);
	}

	void Film::setImage(const Spectrum *img) const
	{
		int nPixels = m_croppedPixelBounds.area();
		for (int i = 0; i < nPixels; ++i) 
		{
			Pixel &p = m_pixels[i];
			img[i].toXYZ(p.m_xyz);
			p.m_filterWeightSum = 1;
			p.m_splatXYZ[0] = p.m_splatXYZ[1] = p.m_splatXYZ[2] = 0;
		}
	}

	void Film::addSplat(const Vec2f &p, Spectrum v)
	{
		

		if (v.hasNaNs()) 
		{
			LOG(ERROR) << stringPrintf("Ignoring splatted spectrum with NaN values "
				"at (%f, %f)", p.x, p.y);
			return;
		}
		else if (v.luminance() < 0.) 
		{
			LOG(ERROR) << stringPrintf("Ignoring splatted spectrum with negative "
				"luminance %f at (%f, %f)", v.luminance(), p.x, p.y);
			return;
		}
		else if (glm::isinf(v.luminance())) 
		{
			LOG(ERROR) << stringPrintf("Ignoring splatted spectrum with infinite "
				"luminance at (%f, %f)", p.x, p.y);
			return;
		}

		Vec2i pi = Vec2i(floor(p));
		if (!insideExclusive(pi, m_croppedPixelBounds)) 
			return;

		//限制亮度
		if (v.luminance() > m_maxSampleLuminance)
		{
			v *= m_maxSampleLuminance / v.luminance();
		}

		Float xyz[3];
		v.toXYZ(xyz);

		Pixel &pixel = getPixel(pi);
		for (int i = 0; i < 3; ++i)
		{
			pixel.m_splatXYZ[i].add(xyz[i]);
		}
	}

	void Film::clear()
	{
		for (Vec2i p : m_croppedPixelBounds) 
		{
			Pixel &pixel = getPixel(p);
			for (int c = 0; c < 3; ++c)
			{
				pixel.m_splatXYZ[c] = pixel.m_xyz[c] = 0;
			}
			pixel.m_filterWeightSum = 0;
		}
	}
}