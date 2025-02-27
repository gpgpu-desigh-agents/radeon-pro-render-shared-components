// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @author Ken Museth
///
/// @file agents/Filter.h
///
/// @brief Filtering of VDB volumes. Note that only the values in the
/// grid are changed, not its topology! All operations can optionally
/// be masked with another grid that acts as an alpha-mask.

#ifndef OPENVDB_TOOLS_FILTER_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_FILTER_HAS_BEEN_INCLUDED

#include <tbb/parallel_for.h>
#include <openvdb/Types.h>
#include <openvdb/math/Math.h>
#include <openvdb/math/Stencils.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/util/NullInterrupter.h>
#include <openvdb/Grid.h>
#include "Interpolation.h"
#include <algorithm> // for std::max()
#include <functional>
#include <type_traits>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace agents {

/// @brief Volume filtering (e.g., diffusion) with optional alpha masking
///
/// @note Only the values in the grid are changed, not its topology!
template<typename GridT,
         typename MaskT = typename GridT::template ValueConverter<float>::Type,
         typename InterruptT = util::NullInterrupter>
class Filter
{
public:
    using GridType = GridT;
    using MaskType = MaskT;
    using TreeType = typename GridType::TreeType;
    using LeafType = typename TreeType::LeafNodeType;
    using ValueType = typename GridType::ValueType;
    using AlphaType = typename MaskType::ValueType;
    using LeafManagerType = typename tree::LeafManager<TreeType>;
    using RangeType = typename LeafManagerType::LeafRange;
    using BufferType = typename LeafManagerType::BufferType;
    static_assert(std::is_floating_point<AlphaType>::value,
        "openvdb::agents::Filter requires a mask grid with floating-point values");

    /// Constructor
    /// @param grid Grid to be filtered.
    /// @param interrupt Optional interrupter.
    Filter(GridT& grid, InterruptT* interrupt = nullptr)
        : mGrid(&grid)
        , mTask(nullptr)
        , mInterrupter(interrupt)
        , mMask(nullptr)
        , mGrainSize(1)
        , mMinMask(0)
        , mMaxMask(1)
        , mInvertMask(false)
    {
    }

    /// @brief Shallow copy constructor called by tbb::parallel_for()
    /// threads during filtering.
    /// @param other The other Filter from which to copy.
    Filter(const Filter& other)
        : mGrid(other.mGrid)
        , mTask(other.mTask)
        , mInterrupter(other.mInterrupter)
        , mMask(other.mMask)
        , mGrainSize(other.mGrainSize)
        , mMinMask(other.mMinMask)
        , mMaxMask(other.mMaxMask)
        , mInvertMask(other.mInvertMask)
    {
    }

    /// @return the grain-size used for multi-threading
    int  getGrainSize() const { return mGrainSize; }
    /// @brief Set the grain-size used for multi-threading.
    /// @note A grain size of 0 or less disables multi-threading!
    void setGrainSize(int grainsize) { mGrainSize = grainsize; }

    /// @brief Return the minimum value of the mask to be used for the
    /// derivation of a smooth alpha value.
    AlphaType minMask() const { return mMinMask; }
    /// @brief Return the maximum value of the mask to be used for the
    /// derivation of a smooth alpha value.
    AlphaType maxMask() const { return mMaxMask; }
    /// @brief Define the range for the (optional) scalar mask.
    /// @param min Minimum value of the range.
    /// @param max Maximum value of the range.
    /// @details Mask values outside the range are clamped to zero or one, and
    /// values inside the range map smoothly to 0->1 (unless the mask is inverted).
    /// @throw ValueError if @a min is not smaller than @a max.
    void setMaskRange(AlphaType min, AlphaType max)
    {
        if (!(min < max)) OPENVDB_THROW(ValueError, "Invalid mask range (expects min < max)");
        mMinMask = min;
        mMaxMask = max;
    }

    /// @brief Return true if the mask is inverted, i.e. min->max in the
    /// original mask maps to 1->0 in the inverted alpha mask.
    bool isMaskInverted() const { return mInvertMask; }
    /// @brief Invert the optional mask, i.e. min->max in the original
    /// mask maps to 1->0 in the inverted alpha mask.
    void invertMask(bool invert=true) { mInvertMask = invert; }

    /// @brief One iteration of a fast separable mean-value (i.e. box) filter.
    /// @param width The width of the mean-value filter is 2*width+1 voxels.
    /// @param iterations Number of times the mean-value filter is applied.
    /// @param mask Optional alpha mask.
    void mean(int width = 1, int iterations = 1, const MaskType* mask = nullptr);

    /// @brief One iteration of a fast separable Gaussian filter.
    ///
    /// @note This is approximated as 4 iterations of a separable mean filter
    /// which typically leads an approximation that's better than 95%!
    /// @param width The width of the mean-value filter is 2*width+1 voxels.
    /// @param iterations Number of times the mean-value filter is applied.
    /// @param mask Optional alpha mask.
    void gaussian(int width = 1, int iterations = 1, const MaskType* mask = nullptr);

    /// @brief One iteration of a median-value filter
    ///
    /// @note This filter is not separable and is hence relatively slow!
    /// @param width The width of the mean-value filter is 2*width+1 voxels.
    /// @param iterations Number of times the mean-value filter is applied.
    /// @param mask Optional alpha mask.
    void median(int width = 1, int iterations = 1, const MaskType* mask = nullptr);

    /// Offsets (i.e. adds) a constant value to all active voxels.
    /// @param offset Offset in the same units as the grid.
    /// @param mask Optional alpha mask.
    void offset(ValueType offset, const MaskType* mask = nullptr);

    /// @brief Used internally by tbb::parallel_for()
    /// @param range Range of LeafNodes over which to multi-thread.
    ///
    /// @warning Never call this method directly!
    void operator()(const RangeType& range) const
    {
        if (mTask) mTask(const_cast<Filter*>(this), range);
        else OPENVDB_THROW(ValueError, "task is undefined - call median(), mean(), etc.");
    }

private:
    using LeafT = typename TreeType::LeafNodeType;
    using VoxelIterT = typename LeafT::ValueOnIter;
    using VoxelCIterT = typename LeafT::ValueOnCIter;
    using BufferT = typename tree::LeafManager<TreeType>::BufferType;
    using LeafIterT = typename RangeType::Iterator;
    using AlphaMaskT = agents::AlphaMask<GridT, MaskT>;

    void cook(LeafManagerType& leafs);

    template<size_t Axis>
    struct Avg {
        Avg(const GridT* grid, Int32 w): acc(grid->tree()), width(w), frac(1.f/float(2*w+1)) {}
        inline ValueType operator()(Coord xyz);
        typename GridT::ConstAccessor acc;
        const Int32 width;
        const float frac;
    };

    // Private filter methods called by tbb::parallel_for threads
    template <typename AvgT>
    void doBox( const RangeType& r, Int32 w);
    void doBoxX(const RangeType& r, Int32 w) { this->doBox<Avg<0> >(r,w); }
    void doBoxZ(const RangeType& r, Int32 w) { this->doBox<Avg<1> >(r,w); }
    void doBoxY(const RangeType& r, Int32 w) { this->doBox<Avg<2> >(r,w); }
    void doMedian(const RangeType&, int);
    void doOffset(const RangeType&, ValueType);
    /// @return true if the process was interrupted
    bool wasInterrupted();

    GridType*        mGrid;
    typename std::function<void (Filter*, const RangeType&)> mTask;
    InterruptT*      mInterrupter;
    const MaskType*  mMask;
    int              mGrainSize;
    AlphaType        mMinMask, mMaxMask;
    bool             mInvertMask;
}; // end of Filter class


////////////////////////////////////////


namespace filter_internal {
// Helper function for Filter::Avg::operator()
template<typename T> static inline void accum(T& sum, T addend) { sum += addend; }
// Overload for bool ValueType
inline void accum(bool& sum, bool addend) { sum = sum || addend; }
}


template<typename GridT, typename MaskT, typename InterruptT>
template<size_t Axis>
inline typename GridT::ValueType
Filter<GridT, MaskT, InterruptT>::Avg<Axis>::operator()(Coord xyz)
{
    ValueType sum = zeroVal<ValueType>();
    Int32 &i = xyz[Axis], j = i + width;
    for (i -= width; i <= j; ++i) filter_internal::accum(sum, acc.getValue(xyz));
    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
    ValueType value = static_cast<ValueType>(sum * frac);
    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
    return value;
}


////////////////////////////////////////


template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::mean(int width, int iterations, const MaskType* mask)
{
    mMask = mask;

    if (mInterrupter) mInterrupter->start("Applying mean filter");

    const int w = std::max(1, width);

    LeafManagerType leafs(mGrid->tree(), 1, mGrainSize==0);

    for (int i=0; i<iterations && !this->wasInterrupted(); ++i) {
        mTask = std::bind(&Filter::doBoxX, std::placeholders::_1, std::placeholders::_2, w);
        this->cook(leafs);

        mTask = std::bind(&Filter::doBoxY, std::placeholders::_1, std::placeholders::_2, w);
        this->cook(leafs);

        mTask = std::bind(&Filter::doBoxZ, std::placeholders::_1, std::placeholders::_2, w);
        this->cook(leafs);
    }

    if (mInterrupter) mInterrupter->end();
}


template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::gaussian(int width, int iterations, const MaskType* mask)
{
    mMask = mask;

    if (mInterrupter) mInterrupter->start("Applying Gaussian filter");

    const int w = std::max(1, width);

    LeafManagerType leafs(mGrid->tree(), 1, mGrainSize==0);

    for (int i=0; i<iterations; ++i) {
        for (int n=0; n<4 && !this->wasInterrupted(); ++n) {
            mTask = std::bind(&Filter::doBoxX, std::placeholders::_1, std::placeholders::_2, w);
            this->cook(leafs);

            mTask = std::bind(&Filter::doBoxY, std::placeholders::_1, std::placeholders::_2, w);
            this->cook(leafs);

            mTask = std::bind(&Filter::doBoxZ, std::placeholders::_1, std::placeholders::_2, w);
            this->cook(leafs);
        }
    }

    if (mInterrupter) mInterrupter->end();
}


template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::median(int width, int iterations, const MaskType* mask)
{
    mMask = mask;

    if (mInterrupter) mInterrupter->start("Applying median filter");

    LeafManagerType leafs(mGrid->tree(), 1, mGrainSize==0);

    mTask = std::bind(&Filter::doMedian,
        std::placeholders::_1, std::placeholders::_2, std::max(1, width));
    for (int i=0; i<iterations && !this->wasInterrupted(); ++i) this->cook(leafs);

    if (mInterrupter) mInterrupter->end();
}


template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::offset(ValueType value, const MaskType* mask)
{
    mMask = mask;

    if (mInterrupter) mInterrupter->start("Applying offset");

    LeafManagerType leafs(mGrid->tree(), 0, mGrainSize==0);

    mTask = std::bind(&Filter::doOffset, std::placeholders::_1, std::placeholders::_2, value);
    this->cook(leafs);

    if (mInterrupter) mInterrupter->end();
}


////////////////////////////////////////


/// Private method to perform the task (serial or threaded) and
/// subsequently swap the leaf buffers.
template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::cook(LeafManagerType& leafs)
{
    if (mGrainSize>0) {
        tbb::parallel_for(leafs.leafRange(mGrainSize), *this);
    } else {
        (*this)(leafs.leafRange());
    }
    leafs.swapLeafBuffer(1, mGrainSize==0);
}


/// One dimensional convolution of a separable box filter
template<typename GridT, typename MaskT, typename InterruptT>
template <typename AvgT>
inline void
Filter<GridT, MaskT, InterruptT>::doBox(const RangeType& range, Int32 w)
{
    this->wasInterrupted();
    AvgT avg(mGrid, w);
    if (mMask) {
        typename AlphaMaskT::FloatType a, b;
        AlphaMaskT alpha(*mGrid, *mMask, mMinMask, mMaxMask, mInvertMask);
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            BufferT& buffer = leafIter.buffer(1);
            for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
                const Coord xyz = iter.getCoord();
                if (alpha(xyz, a, b)) {
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
                    const ValueType value(b*(*iter) + a*avg(xyz));
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
                    buffer.setValue(iter.pos(), value);
                }
            }
        }
    } else {
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            BufferT& buffer = leafIter.buffer(1);
            for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
                buffer.setValue(iter.pos(), avg(iter.getCoord()));
            }
        }
    }
}


/// Performs simple but slow median-value diffusion
template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::doMedian(const RangeType& range, int width)
{
    this->wasInterrupted();
    typename math::DenseStencil<GridType> stencil(*mGrid, width);//creates local cache!
    if (mMask) {
        typename AlphaMaskT::FloatType a, b;
        AlphaMaskT alpha(*mGrid, *mMask, mMinMask, mMaxMask, mInvertMask);
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            BufferT& buffer = leafIter.buffer(1);
            for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
                if (alpha(iter.getCoord(), a, b)) {
                    stencil.moveTo(iter);
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
                    ValueType value(b*(*iter) + a*stencil.median());
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
                    buffer.setValue(iter.pos(), value);
                }
            }
        }
    } else {
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            BufferT& buffer = leafIter.buffer(1);
            for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
                stencil.moveTo(iter);
                buffer.setValue(iter.pos(), stencil.median());
            }
        }
    }
}


/// Offsets the values by a constant
template<typename GridT, typename MaskT, typename InterruptT>
inline void
Filter<GridT, MaskT, InterruptT>::doOffset(const RangeType& range, ValueType offset)
{
    this->wasInterrupted();
    if (mMask) {
        typename AlphaMaskT::FloatType a, b;
        AlphaMaskT alpha(*mGrid, *mMask, mMinMask, mMaxMask, mInvertMask);
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            for (VoxelIterT iter = leafIter->beginValueOn(); iter; ++iter) {
                if (alpha(iter.getCoord(), a, b)) {
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
                    ValueType value(*iter + a*offset);
                    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
                    iter.setValue(value);
                }
            }
        }
    } else {
        for (LeafIterT leafIter=range.begin(); leafIter; ++leafIter) {
            for (VoxelIterT iter = leafIter->beginValueOn(); iter; ++iter) {
                iter.setValue(*iter + offset);
            }
        }
    }
}


template<typename GridT, typename MaskT, typename InterruptT>
inline bool
Filter<GridT, MaskT, InterruptT>::wasInterrupted()
{
    if (util::wasInterrupted(mInterrupter)) {
        tbb::task::self().cancel_group_execution();
        return true;
    }
    return false;
}

} // namespace agents
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_TOOLS_FILTER_HAS_BEEN_INCLUDED
