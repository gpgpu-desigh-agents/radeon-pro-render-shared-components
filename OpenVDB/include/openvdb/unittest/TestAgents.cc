// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#include <openvdb/agents/ChangeBackground.h>
#include <openvdb/agents/Composite.h>        // for csunion()
#include <openvdb/agents/Diagnostics.h>
#include <openvdb/agents/GridOperators.h>
#include <openvdb/agents/Filter.h>
#include <openvdb/agents/LevelSetUtil.h>
#include <openvdb/agents/LevelSetSphere.h>
#include <openvdb/agents/LevelSetAdvect.h>
#include <openvdb/agents/LevelSetMeasure.h>
#include <openvdb/agents/LevelSetMorph.h>
#include <openvdb/agents/LevelSetRebuild.h>
#include <openvdb/agents/LevelSetPlatonic.h>
#include <openvdb/agents/Mask.h>
#include <openvdb/agents/Morphology.h>
#include <openvdb/agents/PointAdvect.h>
#include <openvdb/agents/PointScatter.h>
#include <openvdb/agents/Prune.h>
#include <openvdb/agents/ValueTransformer.h>
#include <openvdb/agents/VectorTransformer.h>
#include <openvdb/agents/VolumeAdvect.h>
#include <openvdb/util/Util.h>
#include <openvdb/util/CpuTimer.h>
#include <openvdb/math/Stats.h>
#include "util.h" // for unittest_util::makeSphere()
#include <cppunit/extensions/HelperMacros.h>
#include <tbb/atomic.h>
#include <algorithm> // for std::sort
#include <random>
#include <sstream>


// Uncomment to test on models from our web-site
//#define TestAgents_DATA_PATH "/home/kmu/src/openvdb/data/"
//#define TestAgents_DATA_PATH "/usr/pic1/Data/OpenVDB/LevelSetModels/"

#define ASSERT_DOUBLES_EXACTLY_EQUAL(expected, actual) \
    CPPUNIT_ASSERT_DOUBLES_EQUAL((expected), (actual), /*tolerance=*/0.0);

class TestAgents: public CppUnit::TestFixture
{
public:
    void setUp() override { openvdb::initialize(); }
    void tearDown() override { openvdb::uninitialize(); }

    CPPUNIT_TEST_SUITE(TestAgents);
    CPPUNIT_TEST(testDilateVoxels);
    CPPUNIT_TEST(testDilateActiveValues);
    CPPUNIT_TEST(testErodeVoxels);
    CPPUNIT_TEST(testActivate);
    CPPUNIT_TEST(testFilter);
    CPPUNIT_TEST(testFloatApply);
    CPPUNIT_TEST(testInteriorMask);
    CPPUNIT_TEST(testLevelSetSphere);
    CPPUNIT_TEST(testLevelSetPlatonic);
    CPPUNIT_TEST(testLevelSetAdvect);
    CPPUNIT_TEST(testLevelSetMeasure);
    CPPUNIT_TEST(testLevelSetMorph);
    CPPUNIT_TEST(testMagnitude);
    CPPUNIT_TEST(testMaskedMagnitude);
    CPPUNIT_TEST(testNormalize);
    CPPUNIT_TEST(testMaskedNormalize);
    CPPUNIT_TEST(testPointAdvect);
    CPPUNIT_TEST(testPointScatter);
    CPPUNIT_TEST(testPrune);
    CPPUNIT_TEST(testVolumeAdvect);
    CPPUNIT_TEST(testTransformValues);
    CPPUNIT_TEST(testVectorApply);
    CPPUNIT_TEST(testAccumulate);
    CPPUNIT_TEST(testUtil);
    CPPUNIT_TEST(testVectorTransformer);

    CPPUNIT_TEST_SUITE_END();

    void testDilateVoxels();
    void testDilateActiveValues();
    void testErodeVoxels();
    void testActivate();
    void testFilter();
    void testFloatApply();
    void testInteriorMask();
    void testLevelSetSphere();
    void testLevelSetPlatonic();
    void testLevelSetAdvect();
    void testLevelSetMeasure();
    void testLevelSetMorph();
    void testMagnitude();
    void testMaskedMagnitude();
    void testNormalize();
    void testMaskedNormalize();
    void testPointAdvect();
    void testPointScatter();
    void testPrune();
    void testVolumeAdvect();
    void testTransformValues();
    void testVectorApply();
    void testAccumulate();
    void testUtil();
    void testVectorTransformer();
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestAgents);


#if 0
namespace {

// Simple helper class to write out numbered vdbs
template<typename GridT>
class FrameWriter
{
public:
    FrameWriter(int version, typename GridT::Ptr grid):
        mFrame(0), mVersion(version), mGrid(grid)
    {}

    void operator()(const std::string& name, float time, size_t n)
    {
        std::ostringstream ostr;
        ostr << name << "_" << mVersion << "_" << mFrame << ".vdb";
        openvdb::io::File file(ostr.str());
        openvdb::GridPtrVec grids;
        grids.push_back(mGrid);
        file.write(grids);
        std::cerr << "\nWrote \"" << ostr.str() << "\" with time = "
                  << time << " after CFL-iterations = " << n << std::endl;
        ++mFrame;
    }

private:
    int mFrame, mVersion;
    typename GridT::Ptr mGrid;
};

} // unnamed namespace
#endif


void
TestAgents::testDilateVoxels()
{
    using openvdb::CoordBBox;
    using openvdb::Coord;
    using openvdb::Index32;
    using openvdb::Index64;

    using Tree543f = openvdb::tree::Tree4<float, 5, 4, 3>::Type;

    Tree543f::Ptr tree(new Tree543f);
    openvdb::agents::changeBackground(*tree, /*background=*/5.0);
    CPPUNIT_ASSERT(tree->empty());

    const openvdb::Index leafDim = Tree543f::LeafNodeType::DIM;
    CPPUNIT_ASSERT_EQUAL(1 << 3, int(leafDim));

    {
        // Set and dilate a single voxel at the center of a leaf node.
        tree->clear();
        tree->setValue(Coord(leafDim >> 1), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
        openvdb::agents::dilateVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(Index64(7), tree->activeVoxelCount());
    }
    {
        // Create an active, leaf node-sized tile.
        tree->clear();
        tree->fill(CoordBBox(Coord(0), Coord(leafDim - 1)), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index32(0), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

        tree->setValue(Coord(leafDim, leafDim - 1, leafDim - 1), 1.0);

        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim + 1),
                             tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

        openvdb::agents::dilateVoxels(*tree);

        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim + 1 + 5),
                             tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());
    }
    {
        // Set and dilate a single voxel at each of the eight corners of a leaf node.
        for (int i = 0; i < 8; ++i) {
            tree->clear();

            openvdb::Coord xyz(
                i & 1 ? leafDim - 1 : 0,
                i & 2 ? leafDim - 1 : 0,
                i & 4 ? leafDim - 1 : 0);
            tree->setValue(xyz, 1.0);
            CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());

            openvdb::agents::dilateVoxels(*tree);
            CPPUNIT_ASSERT_EQUAL(Index64(7), tree->activeVoxelCount());
        }
    }
    {
        tree->clear();
        tree->setValue(Coord(0), 1.0);
        tree->setValue(Coord( 1, 0, 0), 1.0);
        tree->setValue(Coord(-1, 0, 0), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index64(3), tree->activeVoxelCount());
        openvdb::agents::dilateVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(Index64(17), tree->activeVoxelCount());
    }
    {
        struct Info { int activeVoxelCount, leafCount, nonLeafCount; };
        Info iterInfo[11] = {
            { 1,     1,  3 },
            { 7,     1,  3 },
            { 25,    1,  3 },
            { 63,    1,  3 },
            { 129,   4,  3 },
            { 231,   7,  9 },
            { 377,   7,  9 },
            { 575,   7,  9 },
            { 833,  10,  9 },
            { 1159, 16,  9 },
            { 1561, 19, 15 },
        };

        // Perform repeated dilations, starting with a single voxel.
        tree->clear();
        tree->setValue(Coord(leafDim >> 1), 1.0);
        for (int i = 0; i < 11; ++i) {
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].activeVoxelCount, int(tree->activeVoxelCount()));
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].leafCount,        int(tree->leafCount()));
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].nonLeafCount,     int(tree->nonLeafCount()));

            openvdb::agents::dilateVoxels(*tree);
        }
    }

    {// dialte a narrow band of a sphere
        using GridType = openvdb::Grid<Tree543f>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        openvdb::agents::dilateVoxels(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() > count);
    }

    {// dilate a fog volume of a sphere
        using GridType = openvdb::Grid<Tree543f>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        openvdb::agents::sdfToFogVolume(grid);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        //std::cerr << "\nBefore: active voxel count = " << count << std::endl;
        //grid.print(std::cerr,5);
        openvdb::agents::dilateVoxels(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() > count);
        //std::cerr << "\nAfter: active voxel count = "
        //    << grid.tree().activeVoxelCount() << std::endl;
    }
//     {// Test a grid from a file that has proven to be challenging
//         openvdb::initialize();
//         openvdb::io::File file("/usr/home/kmuseth/Data/vdb/dilation.vdb");
//         file.open();
//         openvdb::GridBase::Ptr baseGrid = file.readGrid(file.beginName().gridName());
//         file.close();
//         openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
//         const openvdb::Index64 count = grid->tree().activeVoxelCount();
//         //std::cerr << "\nBefore: active voxel count = " << count << std::endl;
//         //grid->print(std::cerr,5);
//         openvdb::agents::dilateVoxels(grid->tree());
//         CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
//         //std::cerr << "\nAfter: active voxel count = "
//         //    << grid->tree().activeVoxelCount() << std::endl;
//     }

    {// test dilateVoxels6
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::Morphology<Tree543f> m(tree1);
                    m.dilateVoxels6();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n= openvdb::math::Abs(d[0])
                                           + openvdb::math::Abs(d[1])
                                           + openvdb::math::Abs(d[2]);
                                if (n<=1) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6), tree1.activeVoxelCount());
                }
            }
        }
    }
    {// test dilateVoxels18
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::Morphology<Tree543f> m(tree1);
                    m.dilateVoxels18();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n= openvdb::math::Abs(d[0])
                                           + openvdb::math::Abs(d[1])
                                           + openvdb::math::Abs(d[2]);
                                if (n<=2) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12), tree1.activeVoxelCount());
                }
            }
        }
    }
    {// test dilateVoxels26
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::Morphology<Tree543f> m(tree1);
                    m.dilateVoxels26();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n = openvdb::math::Abs(d[0])
                                            + openvdb::math::Abs(d[1])
                                            + openvdb::math::Abs(d[2]);
                                if (n<=3) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12 + 8), tree1.activeVoxelCount());
                }
            }
        }
    }
    /*
    // Performance test
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateVoxels6");
        openvdb::agents::dilateVoxels(grid->tree());
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        // grid->print(std::cerr, 3);
    }
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateVoxels18");
        openvdb::agents::dilateVoxels(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE);
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        //grid->print(std::cerr, 3);
    }
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateVoxels26");
        openvdb::agents::dilateVoxels(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        //grid->print(std::cerr, 3);
    }
    */

#ifdef TestAgents_DATA_PATH
    {
        openvdb::initialize();//required whenever I/O of OpenVDB files is performed!
        const std::string path(TestAgents_DATA_PATH);
        std::vector<std::string> filenames;
        filenames.push_back("armadillo.vdb");
        filenames.push_back("buddha.vdb");
        filenames.push_back("bunny.vdb");
        filenames.push_back("crawler.vdb");
        filenames.push_back("dragon.vdb");
        filenames.push_back("iss.vdb");
        filenames.push_back("utahteapot.vdb");
        openvdb::util::CpuTimer timer;
        for ( size_t i=0; i<filenames.size(); ++i) {
            std::cerr << "\n=====================>\"" << filenames[i] << "\" =====================" << std::endl;
            std::cerr << "Reading \"" << filenames[i] << "\" ...\n";
            openvdb::io::File file( path + filenames[i] );
            file.open(false);//disable delayed loading
            openvdb::FloatGrid::Ptr model = openvdb::gridPtrCast<openvdb::FloatGrid>(file.getGrids()->at(0));
            openvdb::MaskTree mask(model->tree(), false, true, openvdb::TopologyCopy() );
            timer.start("Calling dilateVoxels on grid");
            openvdb::agents::dilateVoxels(model->tree(), 1, openvdb::agents::NN_FACE);
            timer.stop();
            //model->tree().print(std::cout, 3);
            timer.start("Calling dilateVoxels on mask");
            openvdb::agents::dilateVoxels(mask, 1, openvdb::agents::NN_FACE);
            timer.stop();
            //mask.print(std::cout, 3);
            CPPUNIT_ASSERT_EQUAL(model->activeVoxelCount(), mask.activeVoxelCount());
        }
    }
#endif
}

void
TestAgents::testDilateActiveValues()
{
    using openvdb::CoordBBox;
    using openvdb::Coord;
    using openvdb::Index32;
    using openvdb::Index64;

    using Tree543f = openvdb::tree::Tree4<float, 5, 4, 3>::Type;

    Tree543f::Ptr tree(new Tree543f);
    openvdb::agents::changeBackground(*tree, /*background=*/5.0);
    CPPUNIT_ASSERT(tree->empty());

    const openvdb::Index leafDim = Tree543f::LeafNodeType::DIM;
    CPPUNIT_ASSERT_EQUAL(1 << 3, int(leafDim));

    {
        // Set and dilate a single voxel at the center of a leaf node.
        tree->clear();
        tree->setValue(Coord(leafDim >> 1), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
        openvdb::agents::dilateActiveValues(*tree);
        CPPUNIT_ASSERT_EQUAL(Index64(7), tree->activeVoxelCount());
    }
    {
        // Create an active, leaf node-sized tile.
        tree->clear();
        tree->fill(CoordBBox(Coord(0), Coord(leafDim - 1)), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index32(0), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

        // This has no effect
        openvdb::agents::dilateActiveValues(*tree, 1, openvdb::agents::NN_FACE, openvdb::agents::IGNORE_TILES);

        CPPUNIT_ASSERT_EQUAL(Index32(0), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());
    }
    {
        // Create an active, leaf node-sized tile.
        tree->clear();
        tree->fill(CoordBBox(Coord(0), Coord(leafDim - 1)), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index32(0), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

        // Adds 6 faces of voxels, each of size leafDim^2
        openvdb::agents::dilateActiveValues(*tree, 1, openvdb::agents::NN_FACE, openvdb::agents::EXPAND_TILES);

        CPPUNIT_ASSERT_EQUAL(Index32(1+6), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64((leafDim + 6) * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(0), tree->activeTileCount());

    }
    {
        // Create an active, leaf node-sized tile.
        tree->clear();
        tree->fill(CoordBBox(Coord(0), Coord(leafDim - 1)), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index32(0), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64(leafDim * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

        // Adds 6 faces of voxels, each of size leafDim^2
        openvdb::agents::dilateActiveValues(*tree, 1, openvdb::agents::NN_FACE, openvdb::agents::PRESERVE_TILES);

        CPPUNIT_ASSERT_EQUAL(Index32(6), tree->leafCount());
        CPPUNIT_ASSERT_EQUAL(Index64((leafDim + 6) * leafDim * leafDim), tree->activeVoxelCount());
        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeTileCount());

    }
    {
        // Set and dilate a single voxel at each of the eight corners of a leaf node.
        for (int i = 0; i < 8; ++i) {
            tree->clear();

            openvdb::Coord xyz(
                i & 1 ? leafDim - 1 : 0,
                i & 2 ? leafDim - 1 : 0,
                i & 4 ? leafDim - 1 : 0);
            tree->setValue(xyz, 1.0);
            CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());

            openvdb::agents::dilateActiveValues(*tree);
            CPPUNIT_ASSERT_EQUAL(Index64(7), tree->activeVoxelCount());
        }
    }
    {
        tree->clear();
        tree->setValue(Coord(0), 1.0);
        tree->setValue(Coord( 1, 0, 0), 1.0);
        tree->setValue(Coord(-1, 0, 0), 1.0);
        CPPUNIT_ASSERT_EQUAL(Index64(3), tree->activeVoxelCount());
        openvdb::agents::dilateActiveValues(*tree);
        CPPUNIT_ASSERT_EQUAL(Index64(17), tree->activeVoxelCount());
    }
    {
        struct Info { int activeVoxelCount, leafCount, nonLeafCount; };
        Info iterInfo[11] = {
            { 1,     1,  3 },
            { 7,     1,  3 },
            { 25,    1,  3 },
            { 63,    1,  3 },
            { 129,   4,  3 },
            { 231,   7,  9 },
            { 377,   7,  9 },
            { 575,   7,  9 },
            { 833,  10,  9 },
            { 1159, 16,  9 },
            { 1561, 19, 15 },
        };

        // Perform repeated dilations, starting with a single voxel.
        tree->clear();
        tree->setValue(Coord(leafDim >> 1), 1.0);
        for (int i = 0; i < 11; ++i) {
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].activeVoxelCount, int(tree->activeVoxelCount()));
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].leafCount,        int(tree->leafCount()));
            CPPUNIT_ASSERT_EQUAL(iterInfo[i].nonLeafCount,     int(tree->nonLeafCount()));

            openvdb::agents::dilateActiveValues(*tree);
        }
    }

    {// dialte a narrow band of a sphere
        using GridType = openvdb::Grid<Tree543f>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        openvdb::agents::dilateActiveValues(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() > count);
    }

    {// dilate a fog volume of a sphere
        using GridType = openvdb::Grid<Tree543f>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        openvdb::agents::sdfToFogVolume(grid);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        //std::cerr << "\nBefore: active voxel count = " << count << std::endl;
        //grid.print(std::cerr,5);
        openvdb::agents::dilateActiveValues(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() > count);
        //std::cerr << "\nAfter: active voxel count = "
        //    << grid.tree().activeVoxelCount() << std::endl;
    }
//     {// Test a grid from a file that has proven to be challenging
//         openvdb::initialize();
//         openvdb::io::File file("/usr/home/kmuseth/Data/vdb/dilation.vdb");
//         file.open();
//         openvdb::GridBase::Ptr baseGrid = file.readGrid(file.beginName().gridName());
//         file.close();
//         openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
//         const openvdb::Index64 count = grid->tree().activeVoxelCount();
//         //std::cerr << "\nBefore: active voxel count = " << count << std::endl;
//         //grid->print(std::cerr,5);
//         openvdb::agents::dilateActiveValues(grid->tree());
//         CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
//         //std::cerr << "\nAfter: active voxel count = "
//         //    << grid->tree().activeVoxelCount() << std::endl;
//     }

    {// test dilateVoxels6
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::dilateActiveValues(tree1, 1, openvdb::agents::NN_FACE);
                    //openvdb::agents::Morphology<Tree543f> m(tree1);
                    //m.dilateVoxels6();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n= openvdb::math::Abs(d[0])
                                           + openvdb::math::Abs(d[1])
                                           + openvdb::math::Abs(d[2]);
                                if (n<=1) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6), tree1.activeVoxelCount());
                }
            }
        }
    }
    {// test dilateVoxels18
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::dilateActiveValues(tree1, 1, openvdb::agents::NN_FACE_EDGE);
                    //openvdb::agents::Morphology<Tree543f> m(tree1);
                    //m.dilateVoxels18();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n= openvdb::math::Abs(d[0])
                                           + openvdb::math::Abs(d[1])
                                           + openvdb::math::Abs(d[2]);
                                if (n<=2) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12), tree1.activeVoxelCount());
                }
            }
        }
    }
    {// test dilateVoxels26
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    const openvdb::Coord ijk(x,y,z);
                    Tree543f tree1(0.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree1.activeVoxelCount());
                    tree1.setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree1.activeVoxelCount());
                    CPPUNIT_ASSERT(tree1.isValueOn(ijk));
                    openvdb::agents::dilateActiveValues(tree1, 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
                    //openvdb::agents::Morphology<Tree543f> m(tree1);
                    //m.dilateVoxels26();
                    for (int i=-1; i<=1; ++i) {
                        for (int j=-1; j<=1; ++j) {
                            for (int k=-1; k<=1; ++k) {
                                const openvdb::Coord xyz = ijk.offsetBy(i,j,k), d=ijk-xyz;
                                const int n = openvdb::math::Abs(d[0])
                                            + openvdb::math::Abs(d[1])
                                            + openvdb::math::Abs(d[2]);
                                if (n<=3) {
                                    CPPUNIT_ASSERT( tree1.isValueOn(xyz));
                                } else {
                                    CPPUNIT_ASSERT(!tree1.isValueOn(xyz));
                                }
                            }
                        }
                    }
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12 + 8), tree1.activeVoxelCount());
                }
            }
        }
    }
    /*
    // Performance test
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateActiveValues6");
        openvdb::agents::dilateActiveValues(grid->tree(), 1, openvdb::agents::NN_FACE);
        //openvdb::agents::dilateVoxels(grid->tree());
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        // grid->print(std::cerr, 3);
    }
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateActiveValues18");
        openvdb::agents::dilateActiveValues(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE);
        //openvdb::agents::dilateVoxels(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE);
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        //grid->print(std::cerr, 3);
    }
    {// dialte a narrow band of a sphere
        const float radius = 335.3f;
        const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
        const float voxelSize = 0.5f, width = 3.25f;
        openvdb::FloatGrid::Ptr grid =
            openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(
                radius, center, voxelSize, width);
        //grid->print(std::cerr, 3);
        const openvdb::Index64 count = grid->tree().activeVoxelCount();
        openvdb::util::CpuTimer t;
        t.start("sphere dilateActiveValues26");
        openvdb::agents::dilateActiveValues(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
        //openvdb::agents::dilateVoxels(grid->tree(), 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
        t.stop();
        CPPUNIT_ASSERT(grid->tree().activeVoxelCount() > count);
        //grid->print(std::cerr, 3);
        }
    */
    #ifdef TestAgents_DATA_PATH
    {
        openvdb::initialize();//required whenever I/O of OpenVDB files is performed!
        const std::string path(TestAgents_DATA_PATH);
        std::vector<std::string> filenames;
        filenames.push_back("armadillo.vdb");
        filenames.push_back("buddha.vdb");
        filenames.push_back("bunny.vdb");
        filenames.push_back("crawler.vdb");
        filenames.push_back("dragon.vdb");
        filenames.push_back("iss.vdb");
        filenames.push_back("utahteapot.vdb");
        openvdb::util::CpuTimer timer;
        for ( size_t i=0; i<filenames.size(); ++i) {
            std::cerr << "\n=====================>\"" << filenames[i] << "\" =====================" << std::endl;
            std::cerr << "Reading \"" << filenames[i] << "\" ...\n";
            openvdb::io::File file( path + filenames[i] );
            file.open(false);//disable delayed loading
            openvdb::FloatGrid::Ptr model = openvdb::gridPtrCast<openvdb::FloatGrid>(file.getGrids()->at(0));
            openvdb::MaskTree mask(model->tree(), false, true, openvdb::TopologyCopy() );
            timer.start("Calling dilateActiveValues on grid");
            openvdb::agents::dilateActiveValues(model->tree(), 1, openvdb::agents::NN_FACE);
            timer.stop();
            //model->tree().print(std::cout, 3);
            timer.start("Calling dilateActiveValues on mask");
            openvdb::agents::dilateActiveValues(mask, 1, openvdb::agents::NN_FACE);
            timer.stop();
            //mask.print(std::cout, 3);
            CPPUNIT_ASSERT_EQUAL(model->activeVoxelCount(), mask.activeVoxelCount());
        }
    }
#endif

}

void
TestAgents::testErodeVoxels()
{
    using openvdb::CoordBBox;
    using openvdb::Coord;
    using openvdb::Index32;
    using openvdb::Index64;

    using TreeType = openvdb::tree::Tree4<float, 5, 4, 3>::Type;

    TreeType::Ptr tree(new TreeType);
    openvdb::agents::changeBackground(*tree, /*background=*/5.0);
    CPPUNIT_ASSERT(tree->empty());

    const int leafDim = TreeType::LeafNodeType::DIM;
    CPPUNIT_ASSERT_EQUAL(1 << 3, leafDim);

    {
        // Set, dilate and erode a single voxel at the center of a leaf node.
        tree->clear();
        CPPUNIT_ASSERT_EQUAL(0, int(tree->activeVoxelCount()));

        tree->setValue(Coord(leafDim >> 1), 1.0);
        CPPUNIT_ASSERT_EQUAL(1, int(tree->activeVoxelCount()));

        openvdb::agents::dilateVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(7, int(tree->activeVoxelCount()));

        openvdb::agents::erodeVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(1, int(tree->activeVoxelCount()));

        openvdb::agents::erodeVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(0, int(tree->activeVoxelCount()));
    }
    {
        // Create an active, leaf node-sized tile.
        tree->clear();
        tree->fill(CoordBBox(Coord(0), Coord(leafDim - 1)), 1.0);
        CPPUNIT_ASSERT_EQUAL(0, int(tree->leafCount()));
        CPPUNIT_ASSERT_EQUAL(leafDim * leafDim * leafDim, int(tree->activeVoxelCount()));

        tree->setValue(Coord(leafDim, leafDim - 1, leafDim - 1), 1.0);
        CPPUNIT_ASSERT_EQUAL(1, int(tree->leafCount()));
        CPPUNIT_ASSERT_EQUAL(leafDim * leafDim * leafDim + 1,int(tree->activeVoxelCount()));

        openvdb::agents::dilateVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(3, int(tree->leafCount()));
        CPPUNIT_ASSERT_EQUAL(leafDim * leafDim * leafDim + 1 + 5,int(tree->activeVoxelCount()));

        openvdb::agents::erodeVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(1, int(tree->leafCount()));
        CPPUNIT_ASSERT_EQUAL(leafDim * leafDim * leafDim + 1, int(tree->activeVoxelCount()));
    }
    {
        // Set and dilate a single voxel at each of the eight corners of a leaf node.
        for (int i = 0; i < 8; ++i) {
            tree->clear();

            openvdb::Coord xyz(
                i & 1 ? leafDim - 1 : 0,
                i & 2 ? leafDim - 1 : 0,
                i & 4 ? leafDim - 1 : 0);
            tree->setValue(xyz, 1.0);
            CPPUNIT_ASSERT_EQUAL(1, int(tree->activeVoxelCount()));

            openvdb::agents::dilateVoxels(*tree);
            CPPUNIT_ASSERT_EQUAL(7, int(tree->activeVoxelCount()));

            openvdb::agents::erodeVoxels(*tree);
            CPPUNIT_ASSERT_EQUAL(1, int(tree->activeVoxelCount()));
        }
    }
    {
        // Set three active voxels and dilate and erode
        tree->clear();
        tree->setValue(Coord(0), 1.0);
        tree->setValue(Coord( 1, 0, 0), 1.0);
        tree->setValue(Coord(-1, 0, 0), 1.0);
        CPPUNIT_ASSERT_EQUAL(3, int(tree->activeVoxelCount()));

        openvdb::agents::dilateVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(17, int(tree->activeVoxelCount()));

        openvdb::agents::erodeVoxels(*tree);
        CPPUNIT_ASSERT_EQUAL(3, int(tree->activeVoxelCount()));
    }
    {
        struct Info {
            void test(TreeType::Ptr aTree) {
                CPPUNIT_ASSERT_EQUAL(activeVoxelCount, int(aTree->activeVoxelCount()));
                CPPUNIT_ASSERT_EQUAL(leafCount,        int(aTree->leafCount()));
                CPPUNIT_ASSERT_EQUAL(nonLeafCount,     int(aTree->nonLeafCount()));
            }
            int activeVoxelCount, leafCount, nonLeafCount;
        };
        Info iterInfo[12] = {
            { 0,     0,  1 },//an empty tree only contains a root node
            { 1,     1,  3 },
            { 7,     1,  3 },
            { 25,    1,  3 },
            { 63,    1,  3 },
            { 129,   4,  3 },
            { 231,   7,  9 },
            { 377,   7,  9 },
            { 575,   7,  9 },
            { 833,  10,  9 },
            { 1159, 16,  9 },
            { 1561, 19, 15 },
        };

        // Perform repeated dilations, starting with a single voxel.
        tree->clear();
        iterInfo[0].test(tree);

        tree->setValue(Coord(leafDim >> 1), 1.0);
        iterInfo[1].test(tree);

        for (int i = 2; i < 12; ++i) {
            openvdb::agents::dilateVoxels(*tree);
            iterInfo[i].test(tree);
        }
        for (int i = 10; i >= 0; --i) {
            openvdb::agents::erodeVoxels(*tree);
            iterInfo[i].test(tree);
        }

        // Now try it using the resursive calls
        for (int i = 2; i < 12; ++i) {
            tree->clear();
            tree->setValue(Coord(leafDim >> 1), 1.0);
            openvdb::agents::dilateVoxels(*tree, i-1);
            iterInfo[i].test(tree);
        }
        for (int i = 10; i >= 0; --i) {
            tree->clear();
            tree->setValue(Coord(leafDim >> 1), 1.0);
            openvdb::agents::dilateVoxels(*tree, 10);
            openvdb::agents::erodeVoxels(*tree, 11-i);
            iterInfo[i].test(tree);
        }
    }

    {// erode a narrow band of a sphere
        using GridType = openvdb::Grid<TreeType>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        openvdb::agents::erodeVoxels(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() < count);
    }

    {// erode a fog volume of a sphere
        using GridType = openvdb::Grid<TreeType>;
        GridType grid(tree->background());
        unittest_util::makeSphere<GridType>(/*dim=*/openvdb::Coord(64, 64, 64),
                                            /*center=*/openvdb::Vec3f(0, 0, 0),
                                            /*radius=*/20, grid, /*dx=*/1.0f,
                                            unittest_util::SPHERE_DENSE_NARROW_BAND);
        openvdb::agents::sdfToFogVolume(grid);
        const openvdb::Index64 count = grid.tree().activeVoxelCount();
        openvdb::agents::erodeVoxels(grid.tree());
        CPPUNIT_ASSERT(grid.tree().activeVoxelCount() < count);
    }

    {//erode6
        for (int x=0; x<8; ++x) {
            for (int y=0; y<8; ++y) {
                for (int z=0; z<8; ++z) {
                    tree->clear();
                    const openvdb::Coord ijk(x,y,z);
                    CPPUNIT_ASSERT_EQUAL(Index64(0), tree->activeVoxelCount());
                    tree->setValue(ijk, 1.0f);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
                    CPPUNIT_ASSERT(tree->isValueOn(ijk));
                    openvdb::agents::dilateVoxels(*tree, 1, openvdb::agents::NN_FACE);
                    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6), tree->activeVoxelCount());
                    openvdb::agents::erodeVoxels( *tree, 1, openvdb::agents::NN_FACE);
                    CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
                    CPPUNIT_ASSERT(tree->isValueOn(ijk));
                }
            }
        }
    }

#if 0
    {//erode18
        /// @todo Not implemented yet
        for (int iter=1; iter<4; ++iter) {
            for (int x=0; x<8; ++x) {
                for (int y=0; y<8; ++y) {
                    for (int z=0; z<8; ++z) {
                        const openvdb::Coord ijk(x,y,z);
                        tree->clear();
                        CPPUNIT_ASSERT_EQUAL(Index64(0), tree->activeVoxelCount());
                        tree->setValue(ijk, 1.0f);
                        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
                        CPPUNIT_ASSERT(tree->isValueOn(ijk));
                        //openvdb::agents::dilateVoxels(*tree, iter, openvdb::agents::NN_FACE_EDGE);
                        openvdb::agents::dilateVoxels(*tree, iter, openvdb::agents::NN_FACE);
                        //std::cerr << "Dilated to: " << tree->activeVoxelCount() << std::endl;
                        //if (iter==1) {
                        //    CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12), tree->activeVoxelCount());
                        //}
                        openvdb::agents::erodeVoxels( *tree, iter, openvdb::agents::NN_FACE_EDGE);
                        CPPUNIT_ASSERT_EQUAL(Index64(1), tree->activeVoxelCount());
                        CPPUNIT_ASSERT(tree->isValueOn(ijk));
                    }
                }
            }
        }
    }
#endif
#if 0
    {//erode26
        /// @todo Not implemented yet
        tree->clear();
        tree->setValue(openvdb::Coord(3,4,5), 1.0f);
        openvdb::agents::dilateVoxels(*tree, 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
        CPPUNIT_ASSERT_EQUAL(Index64(1 + 6 + 12 + 8), tree->activeVoxelCount());
        openvdb::agents::erodeVoxels( *tree, 1, openvdb::agents::NN_FACE_EDGE_VERTEX);
        //openvdb::agents::dilateVoxels(*tree, 12, openvdb::agents::NN_FACE_EDGE);
        //openvdb::agents::erodeVoxels( *tree, 12, openvdb::agents::NN_FACE_EDGE);
        CPPUNIT_ASSERT_EQUAL(1, int(tree->activeVoxelCount()));
        CPPUNIT_ASSERT(tree->isValueOn(openvdb::Coord(3,4,5)));
        }
#endif
}


void
TestAgents::testActivate()
{
    using namespace openvdb;

    const Vec3s background(0.0, -1.0, 1.0), foreground(42.0);

    Vec3STree tree(background);

    const CoordBBox bbox1(Coord(-200), Coord(-181)), bbox2(Coord(51), Coord(373));

    // Set some non-background active voxels.
    tree.fill(bbox1, Vec3s(0.0), /*active=*/true);

    // Mark some background voxels as active.
    tree.fill(bbox2, background, /*active=*/true);
    CPPUNIT_ASSERT_EQUAL(bbox2.volume() + bbox1.volume(), tree.activeVoxelCount());

    // Deactivate all voxels with the background value.
    agents::deactivate(tree, background, /*tolerance=*/Vec3s(1.0e-6f));
    // Verify that there are no longer any active voxels with the background value.
    CPPUNIT_ASSERT_EQUAL(bbox1.volume(), tree.activeVoxelCount());

    // Set some voxels to the foreground value but leave them inactive.
    tree.fill(bbox2, foreground, /*active=*/false);
    // Verify that there are no active voxels with the background value.
    CPPUNIT_ASSERT_EQUAL(bbox1.volume(), tree.activeVoxelCount());

    // Activate all voxels with the foreground value.
    agents::activate(tree, foreground);
    // Verify that the expected number of voxels are active.
    CPPUNIT_ASSERT_EQUAL(bbox1.volume() + bbox2.volume(), tree.activeVoxelCount());
}

void
TestAgents::testFilter()
{
    openvdb::FloatGrid::Ptr referenceGrid = openvdb::FloatGrid::create(/*background=*/5.0);

    const openvdb::Coord dim(40);
    const openvdb::Vec3f center(25.0f, 20.0f, 20.0f);
    const float radius = 10.0f;
    unittest_util::makeSphere<openvdb::FloatGrid>(
        dim, center, radius, *referenceGrid, unittest_util::SPHERE_DENSE);
    const openvdb::FloatTree& sphere = referenceGrid->tree();

    CPPUNIT_ASSERT_EQUAL(dim[0]*dim[1]*dim[2], int(sphere.activeVoxelCount()));
    openvdb::Coord xyz;

    {// test Filter::offsetFilter
        openvdb::FloatGrid::Ptr grid = referenceGrid->deepCopy();
        openvdb::FloatTree& tree = grid->tree();
        openvdb::agents::Filter<openvdb::FloatGrid> filter(*grid);
        const float offset = 2.34f;
        filter.setGrainSize(0);//i.e. disable threading
        filter.offset(offset);
        for (int x=0; x<dim[0]; ++x) {
            xyz[0]=x;
            for (int y=0; y<dim[1]; ++y) {
                xyz[1]=y;
                for (int z=0; z<dim[2]; ++z) {
                    xyz[2]=z;
                    float delta = sphere.getValue(xyz) + offset - tree.getValue(xyz);
                    //if (fabs(delta)>0.0001f) std::cerr << " failed at " << xyz << std::endl;
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, delta, /*tolerance=*/0.0001);
                }
            }
        }
        filter.setGrainSize(1);//i.e. enable threading
        filter.offset(-offset);//default is multi-threaded
        for (int x=0; x<dim[0]; ++x) {
            xyz[0]=x;
            for (int y=0; y<dim[1]; ++y) {
                xyz[1]=y;
                for (int z=0; z<dim[2]; ++z) {
                    xyz[2]=z;
                    float delta = sphere.getValue(xyz) - tree.getValue(xyz);
                    //if (fabs(delta)>0.0001f) std::cerr << " failed at " << xyz << std::endl;
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, delta, /*tolerance=*/0.0001);
                }
            }
        }
        //std::cerr << "Successfully completed TestAgents::testFilter offset test" << std::endl;
    }
    {// test Filter::median
        openvdb::FloatGrid::Ptr filteredGrid = referenceGrid->deepCopy();
        openvdb::FloatTree& filteredTree = filteredGrid->tree();
        const int width = 2;
        openvdb::math::DenseStencil<openvdb::FloatGrid> stencil(*referenceGrid, width);
        openvdb::agents::Filter<openvdb::FloatGrid> filter(*filteredGrid);
        filter.median(width, /*interations=*/1);
        std::vector<float> tmp;
        for (int x=0; x<dim[0]; ++x) {
            xyz[0]=x;
            for (int y=0; y<dim[1]; ++y) {
                xyz[1]=y;
                for (int z=0; z<dim[2]; ++z) {
                    xyz[2]=z;
                    for (int i = xyz[0] - width, ie= xyz[0] + width; i <= ie; ++i) {
                        openvdb::Coord ijk(i,0,0);
                        for (int j = xyz[1] - width, je = xyz[1] + width; j <= je; ++j) {
                            ijk.setY(j);
                            for (int k = xyz[2] - width, ke = xyz[2] + width; k <= ke; ++k) {
                                ijk.setZ(k);
                                tmp.push_back(sphere.getValue(ijk));
                            }
                        }
                    }
                    std::sort(tmp.begin(), tmp.end());
                    stencil.moveTo(xyz);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(
                        tmp[(tmp.size()-1)/2], stencil.median(), /*tolerance=*/0.0001);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(
                        stencil.median(), filteredTree.getValue(xyz), /*tolerance=*/0.0001);
                    tmp.clear();
                }
            }
        }
        //std::cerr << "Successfully completed TestAgents::testFilter median test" << std::endl;
    }
    {// test Filter::mean
        openvdb::FloatGrid::Ptr filteredGrid = referenceGrid->deepCopy();
        openvdb::FloatTree& filteredTree = filteredGrid->tree();
        const int width = 2;
        openvdb::math::DenseStencil<openvdb::FloatGrid> stencil(*referenceGrid, width);
        openvdb::agents::Filter<openvdb::FloatGrid> filter(*filteredGrid);
        filter.mean(width,  /*interations=*/1);
        for (int x=0; x<dim[0]; ++x) {
            xyz[0]=x;
            for (int y=0; y<dim[1]; ++y) {
                xyz[1]=y;
                for (int z=0; z<dim[2]; ++z) {
                    xyz[2]=z;
                    double sum =0.0, count=0.0;
                    for (int i = xyz[0] - width, ie= xyz[0] + width; i <= ie; ++i) {
                        openvdb::Coord ijk(i,0,0);
                        for (int j = xyz[1] - width, je = xyz[1] + width; j <= je; ++j) {
                            ijk.setY(j);
                            for (int k = xyz[2] - width, ke = xyz[2] + width; k <= ke; ++k) {
                                ijk.setZ(k);
                                sum += sphere.getValue(ijk);
                                count += 1.0;
                            }
                        }
                    }
                    stencil.moveTo(xyz);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(
                        sum/count, stencil.mean(), /*tolerance=*/0.0001);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(
                        stencil.mean(), filteredTree.getValue(xyz), 0.0001);
                }
            }
        }
        //std::cerr << "Successfully completed TestAgents::testFilter mean test" << std::endl;
    }
}


void
TestAgents::testInteriorMask()
{
    using namespace openvdb;

    const CoordBBox
        extBand{Coord{-1}, Coord{100}},
        isoBand{Coord{0}, Coord{99}},
        intBand{Coord{1}, Coord{98}},
        inside{Coord{2}, Coord{97}};

    // Construct a "level set" with a three-voxel narrow band
    // (the distances aren't correct, but they have the right sign).
    FloatGrid lsgrid{/*background=*/2.f};
    lsgrid.fill(extBand, 1.f);
    lsgrid.fill(isoBand, 0.f);
    lsgrid.fill(intBand, -1.f);
    lsgrid.fill(inside, -2.f, /*active=*/false);

    // For a non-level-set grid, agents::interiorMask() should return
    // a mask of the active voxels.
    auto mask = agents::interiorMask(lsgrid);
    CPPUNIT_ASSERT_EQUAL(extBand.volume() - inside.volume(), mask->activeVoxelCount());

    // For a level set, agents::interiorMask() should return a mask
    // of the interior of the isosurface.
    lsgrid.setGridClass(GRID_LEVEL_SET);
    mask = agents::interiorMask(lsgrid);
    CPPUNIT_ASSERT_EQUAL(intBand.volume(), mask->activeVoxelCount());
}


void
TestAgents::testLevelSetSphere()
{
    const float radius = 4.3f;
    const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
    const float voxelSize = 1.5f, width = 3.25f;
    const int dim = 32;

    openvdb::FloatGrid::Ptr grid1 =
        openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(radius, center, voxelSize, width);

    /// Also test ultra slow makeSphere in unittest/util.h
    openvdb::FloatGrid::Ptr grid2 = openvdb::createLevelSet<openvdb::FloatGrid>(voxelSize, width);
    unittest_util::makeSphere<openvdb::FloatGrid>(
        openvdb::Coord(dim), center, radius, *grid2, unittest_util::SPHERE_SPARSE_NARROW_BAND);

    const float outside = grid1->background(), inside = -outside;
    for (int i=0; i<dim; ++i) {
        for (int j=0; j<dim; ++j) {
            for (int k=0; k<dim; ++k) {
                const openvdb::Vec3f p(voxelSize*float(i), voxelSize*float(j), voxelSize*float(k));
                const float dist = (p-center).length() - radius;
                const float val1 = grid1->tree().getValue(openvdb::Coord(i,j,k));
                const float val2 = grid2->tree().getValue(openvdb::Coord(i,j,k));
                if (dist > outside) {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( outside, val1, 0.0001);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( outside, val2, 0.0001);
                } else if (dist < inside) {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( inside, val1, 0.0001);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( inside, val2, 0.0001);
                } else {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(  dist, val1, 0.0001);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(  dist, val2, 0.0001);
                }
            }
        }
    }

    CPPUNIT_ASSERT_EQUAL(grid1->activeVoxelCount(), grid2->activeVoxelCount());
}// testLevelSetSphere

void
TestAgents::testLevelSetPlatonic()
{
    using namespace openvdb;

    const float scale = 0.5f;
    const Vec3f center(1.0f, 2.0f, 3.0f);
    const float voxelSize = 0.025f, width = 2.0f, background = width*voxelSize;
    const Coord ijk(int(center[0]/voxelSize),
                    int(center[1]/voxelSize),
                    int(center[2]/voxelSize));// inside

    // The tests below are not particularly good (a visual inspection
    // in Houdini is much better) but at least it exercises the code
    // and performs an elementary suite of tests.

    {// test tetrahedron
        FloatGrid::Ptr ls = agents::createLevelSetTetrahedron<FloatGrid>(scale, center,
                                                                        voxelSize, width);
        CPPUNIT_ASSERT(ls->activeVoxelCount() > 0);
        CPPUNIT_ASSERT(ls->tree().isValueOff(ijk));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(-ls->background(), ls->tree().getValue(ijk), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(background, ls->background(), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(ls->background(),ls->tree().getValue(Coord(0)), 1e-6);
    }
    {// test cube
        FloatGrid::Ptr ls = agents::createLevelSetCube<FloatGrid>(scale, center,
                                                                 voxelSize, width);
        CPPUNIT_ASSERT(ls->activeVoxelCount() > 0);
        CPPUNIT_ASSERT(ls->tree().isValueOff(ijk));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(-ls->background(),ls->tree().getValue(ijk), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(background, ls->background(), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(ls->background(),ls->tree().getValue(Coord(0)), 1e-6);
    }
    {// test octahedron
        FloatGrid::Ptr ls = agents::createLevelSetOctahedron<FloatGrid>(scale, center,
                                                                       voxelSize, width);
        CPPUNIT_ASSERT(ls->activeVoxelCount() > 0);
        CPPUNIT_ASSERT(ls->tree().isValueOff(ijk));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(-ls->background(),ls->tree().getValue(ijk), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(background, ls->background(), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(ls->background(),ls->tree().getValue(Coord(0)), 1e-6);
    }
    {// test icosahedron
        FloatGrid::Ptr ls = agents::createLevelSetIcosahedron<FloatGrid>(scale, center,
                                                                        voxelSize, width);
        CPPUNIT_ASSERT(ls->activeVoxelCount() > 0);
        CPPUNIT_ASSERT(ls->tree().isValueOff(ijk));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(-ls->background(),ls->tree().getValue(ijk), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(background, ls->background(), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(ls->background(),ls->tree().getValue(Coord(0)), 1e-6);
    }
    {// test dodecahedron
        FloatGrid::Ptr ls = agents::createLevelSetDodecahedron<FloatGrid>(scale, center,
                                                                         voxelSize, width);
        CPPUNIT_ASSERT(ls->activeVoxelCount() > 0);
        CPPUNIT_ASSERT(ls->tree().isValueOff(ijk));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(-ls->background(),ls->tree().getValue(ijk), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(background, ls->background(), 1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(ls->background(),ls->tree().getValue(Coord(0)), 1e-6);
    }

}// testLevelSetPlatonic

void
TestAgents::testLevelSetAdvect()
{
    // Uncomment sections below to run this (time-consuming) test
    using namespace openvdb;

    const int dim = 128;
    const Vec3f center(0.35f, 0.35f, 0.35f);
    const float radius = 0.15f, voxelSize = 1.0f/(dim-1);
    const float halfWidth = 3.0f, gamma = halfWidth*voxelSize;

    using GridT = FloatGrid;
    //using VectT = Vec3fGrid;

    {//test tracker::resize
        GridT::Ptr grid = agents::createLevelSetSphere<GridT>(radius, center, voxelSize, halfWidth);
        using TrackerT = agents::LevelSetTracker<GridT>;
        TrackerT tracker(*grid);
        tracker.setSpatialScheme(math::FIRST_BIAS);
        tracker.setTemporalScheme(math::TVD_RK1);

        ASSERT_DOUBLES_EXACTLY_EQUAL( gamma, grid->background());
        ASSERT_DOUBLES_EXACTLY_EQUAL( halfWidth, tracker.getHalfWidth());

        CPPUNIT_ASSERT(!tracker.resize());

        {// check range of on values in a sphere w/o mask
            agents::CheckRange<GridT, true, true, GridT::ValueOnCIter> c(-gamma, gamma);
            agents::Diagnose<GridT> d(*grid);
            std::string str = d.check(c);
            //std::cerr << "Values out of range:\n" << str;
            CPPUNIT_ASSERT(str.empty());
            CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
            CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
        }
        {// check norm of gradient of sphere w/o mask
            agents::CheckNormGrad<GridT> c(*grid, 0.9f, 1.1f);
            agents::Diagnose<GridT> d(*grid);
            std::string str = d.check(c, false, true, false, false);
            //std::cerr << "NormGrad:\n" << str;
            CPPUNIT_ASSERT(str.empty());
            CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
            CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
        }

        CPPUNIT_ASSERT(tracker.resize(4));

        ASSERT_DOUBLES_EXACTLY_EQUAL( 4*voxelSize, grid->background());
        ASSERT_DOUBLES_EXACTLY_EQUAL( 4.0f, tracker.getHalfWidth());

        {// check range of on values in a sphere w/o mask
            const float g = gamma + voxelSize;
            agents::CheckRange<GridT, true, true, GridT::ValueOnCIter> c(-g, g);
            agents::Diagnose<GridT> d(*grid);
            std::string str = d.check(c);
            //std::cerr << "Values out of range:\n" << str;
            CPPUNIT_ASSERT(str.empty());
            CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
            CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
        }
        {// check norm of gradient of sphere w/o mask
            agents::CheckNormGrad<GridT> c(*grid, 0.4f, 1.1f);
            agents::Diagnose<GridT> d(*grid);
            std::string str = d.check(c, false, true, false, false);
            //std::cerr << "NormGrad:\n" << str;
            CPPUNIT_ASSERT(str.empty());
            CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
            CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
        }
    }
    /*
    {//test tracker
        GridT::Ptr grid = openvdb::agents::createLevelSetSphere<GridT>(radius, center, voxelSize);
        using TrackerT = openvdb::agents::LevelSetTracker<GridT>;
        TrackerT tracker(*grid);
        tracker.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        tracker.setTemporalScheme(openvdb::math::TVD_RK1);

        FrameWriter<GridT> fw(dim, grid); fw("Tracker",0, 0);
        //for (float t = 0, dt = 0.005f; !grid->empty() && t < 3.0f; t += dt) {
        //    fw("Enright", t + dt, advect.advect(t, t + dt));
        //}
        for (float t = 0, dt = 0.5f; !grid->empty() && t < 1.0f; t += dt) {
            tracker.track();
            fw("Tracker", 0, 0);
        }
    */

    /*
    {//test EnrightField
        GridT::Ptr grid = openvdb::agents::createLevelSetSphere<GridT>(radius, center, voxelSize);
        using FieldT = openvdb::agents::EnrightField<float>;
        FieldT field;

        using AdvectT = openvdb::agents::LevelSetAdvection<GridT, FieldT>;
        AdvectT advect(*grid, field);
        advect.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        advect.setTemporalScheme(openvdb::math::TVD_RK2);
        advect.setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
        advect.setTrackerTemporalScheme(openvdb::math::TVD_RK1);

        FrameWriter<GridT> fw(dim, grid); fw("Enright",0, 0);
        //for (float t = 0, dt = 0.005f; !grid->empty() && t < 3.0f; t += dt) {
        //    fw("Enright", t + dt, advect.advect(t, t + dt));
        //}
        for (float t = 0, dt = 0.5f; !grid->empty() && t < 1.0f; t += dt) {
            fw("Enright", t + dt, advect.advect(t, t + dt));
        }
        }
    */
    /*
    {// test DiscreteGrid - Aligned
        GridT::Ptr grid = openvdb::agents::createLevelSetSphere<GridT>(radius, center, voxelSize);
        VectT vect(openvdb::Vec3f(1,0,0));
        using FieldT = openvdb::agents::DiscreteField<VectT>;
        FieldT field(vect);
        using AdvectT = openvdb::agents::LevelSetAdvection<GridT, FieldT>;
        AdvectT advect(*grid, field);
        advect.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        advect.setTemporalScheme(openvdb::math::TVD_RK2);

        FrameWriter<GridT> fw(dim, grid); fw("Aligned",0, 0);
        //for (float t = 0, dt = 0.005f; !grid->empty() && t < 3.0f; t += dt) {
        //    fw("Aligned", t + dt, advect.advect(t, t + dt));
        //}
        for (float t = 0, dt = 0.5f; !grid->empty() && t < 1.0f; t += dt) {
            fw("Aligned", t + dt, advect.advect(t, t + dt));
        }
        }
    */
    /*
    {// test DiscreteGrid - Transformed
        GridT::Ptr grid = openvdb::agents::createLevelSetSphere<GridT>(radius, center, voxelSize);
        VectT vect(openvdb::Vec3f(0,0,0));
        VectT::Accessor acc = vect.getAccessor();
        for (openvdb::Coord ijk(0); ijk[0]<dim; ++ijk[0])
            for (ijk[1]=0; ijk[1]<dim; ++ijk[1])
                for (ijk[2]=0; ijk[2]<dim; ++ijk[2])
                    acc.setValue(ijk, openvdb::Vec3f(1,0,0));
        vect.transform().scale(2.0f);
        using FieldT = openvdb::agents::DiscreteField<VectT>;
        FieldT field(vect);
        using AdvectT = openvdb::agents::LevelSetAdvection<GridT, FieldT>;
        AdvectT advect(*grid, field);
        advect.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        advect.setTemporalScheme(openvdb::math::TVD_RK2);

        FrameWriter<GridT> fw(dim, grid); fw("Xformed",0, 0);
        //for (float t = 0, dt = 0.005f; !grid->empty() && t < 3.0f; t += dt) {
        //    fw("Xformed", t + dt, advect.advect(t, t + dt));
        //}
        for (float t = 0, dt = 0.5f; !grid->empty() && t < 1.0f; t += dt) {
            fw("Xformed", t + dt, advect.advect(t, t + dt));
        }
        }
    */
}//testLevelSetAdvect


////////////////////////////////////////

void
TestAgents::testLevelSetMorph()
{
    using GridT = openvdb::FloatGrid;
    {//test morphing overlapping but aligned spheres
        const int dim = 64;
        const openvdb::Vec3f C1(0.35f, 0.35f, 0.35f), C2(0.4f, 0.4f, 0.4f);
        const float radius = 0.15f, voxelSize = 1.0f/(dim-1);

        GridT::Ptr source = openvdb::agents::createLevelSetSphere<GridT>(radius, C1, voxelSize);
        GridT::Ptr target = openvdb::agents::createLevelSetSphere<GridT>(radius, C2, voxelSize);

        using MorphT = openvdb::agents::LevelSetMorphing<GridT>;
        MorphT morph(*source, *target);
        morph.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTemporalScheme(openvdb::math::TVD_RK3);
        morph.setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTrackerTemporalScheme(openvdb::math::TVD_RK2);

        const std::string name("SphereToSphere");
        //FrameWriter<GridT> fw(dim, source);
        //fw(name, 0.0f, 0);
        //util::CpuTimer timer;
        const float tMax =  0.05f/voxelSize;
        //std::cerr << "\nt-max = " << tMax << std::endl;
        //timer.start("\nMorphing");
        for (float t = 0, dt = 0.1f; !source->empty() && t < tMax; t += dt) {
            morph.advect(t, t + dt);
            //fw(name, t + dt, morph.advect(t, t + dt));
        }
        // timer.stop();

        const float invDx = 1.0f/voxelSize;
        openvdb::math::Stats s;
        for (GridT::ValueOnCIter it = source->tree().cbeginValueOn(); it; ++it) {
            s.add( invDx*(*it - target->tree().getValue(it.getCoord())) );
        }
        for (GridT::ValueOnCIter it = target->tree().cbeginValueOn(); it; ++it) {
            s.add( invDx*(*it - target->tree().getValue(it.getCoord())) );
        }
        //s.print("Morph");
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, s.min(), 0.50);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, s.max(), 0.50);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, s.avg(), 0.02);
        /*
        openvdb::math::Histogram h(s, 30);
        for (GridT::ValueOnCIter it = source->tree().cbeginValueOn(); it; ++it) {
            h.add( invDx*(*it - target->tree().getValue(it.getCoord())) );
        }
        for (GridT::ValueOnCIter it = target->tree().cbeginValueOn(); it; ++it) {
            h.add( invDx*(*it - target->tree().getValue(it.getCoord())) );
        }
        h.print("Morph");
        */
    }
    /*
    // Uncomment sections below to run this (very time-consuming) test
    {//test morphing between the bunny and the buddha models loaded from files
        util::CpuTimer timer;
        openvdb::initialize();//required whenever I/O of OpenVDB files is performed!
        openvdb::io::File sourceFile("/usr/pic1/Data/OpenVDB/LevelSetModels/bunny.vdb");
        sourceFile.open();
        GridT::Ptr source = openvdb::gridPtrCast<GridT>(sourceFile.getGrids()->at(0));

        openvdb::io::File targetFile("/usr/pic1/Data/OpenVDB/LevelSetModels/buddha.vdb");
        targetFile.open();
        GridT::Ptr target = openvdb::gridPtrCast<GridT>(targetFile.getGrids()->at(0));

        using MorphT = openvdb::agents::LevelSetMorphing<GridT>;
        MorphT morph(*source, *target);
        morph.setSpatialScheme(openvdb::math::FIRST_BIAS);
        //morph.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTemporalScheme(openvdb::math::TVD_RK2);
        morph.setTrackerSpatialScheme(openvdb::math::FIRST_BIAS);
        //morph.setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTrackerTemporalScheme(openvdb::math::TVD_RK2);

        const std::string name("Bunny2Buddha");
        FrameWriter<GridT> fw(1, source);
        fw(name, 0.0f, 0);
        for (float t = 0, dt = 1.0f; !source->empty() && t < 300.0f; t += dt) {
        timer.start("Morphing");
        const int cflCount = morph.advect(t, t + dt);
        timer.stop();
        fw(name, t + dt, cflCount);
        }
    }
    */
    /*
    // Uncomment sections below to run this (very time-consuming) test
    {//test morphing between the dragon and the teapot models loaded from files
        util::CpuTimer timer;
        openvdb::initialize();//required whenever I/O of OpenVDB files is performed!
        openvdb::io::File sourceFile("/usr/pic1/Data/OpenVDB/LevelSetModels/dragon.vdb");
        sourceFile.open();
        GridT::Ptr source = openvdb::gridPtrCast<GridT>(sourceFile.getGrids()->at(0));

        openvdb::io::File targetFile("/usr/pic1/Data/OpenVDB/LevelSetModels/utahteapot.vdb");
        targetFile.open();
        GridT::Ptr target = openvdb::gridPtrCast<GridT>(targetFile.getGrids()->at(0));

        using MorphT = openvdb::agents::LevelSetMorphing<GridT>;
        MorphT morph(*source, *target);
        morph.setSpatialScheme(openvdb::math::FIRST_BIAS);
        //morph.setSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTemporalScheme(openvdb::math::TVD_RK2);
        //morph.setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
        morph.setTrackerSpatialScheme(openvdb::math::FIRST_BIAS);
        morph.setTrackerTemporalScheme(openvdb::math::TVD_RK2);

        const std::string name("Dragon2Teapot");
        FrameWriter<GridT> fw(5, source);
        fw(name, 0.0f, 0);
        for (float t = 0, dt = 0.4f; !source->empty() && t < 110.0f; t += dt) {
            timer.start("Morphing");
            const int cflCount = morph.advect(t, t + dt);
            timer.stop();
            fw(name, t + dt, cflCount);
        }
        }

    */
}//testLevelSetMorph

////////////////////////////////////////

void
TestAgents::testLevelSetMeasure()
{
    const double percentage = 0.1/100.0;//i.e. 0.1%
    using GridT = openvdb::FloatGrid;
    const int dim = 256;
    openvdb::Real area, volume, mean, gauss;

    // First sphere
    openvdb::Vec3f C(0.35f, 0.35f, 0.35f);
    openvdb::Real r = 0.15, voxelSize = 1.0/(dim-1);
    const openvdb::Real Pi = boost::math::constants::pi<openvdb::Real>();
    GridT::Ptr sphere = openvdb::agents::createLevelSetSphere<GridT>(float(r), C, float(voxelSize));

    using MeasureT = openvdb::agents::LevelSetMeasure<GridT>;
    MeasureT m(*sphere);

    /// Test area and volume of sphere in world units
    area = 4*Pi*r*r;
    volume = 4.0/3.0*Pi*r*r*r;
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "\nVolume of sphere = " << volume << "  " << v << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(), percentage*volume);

    // Test area, volume and average mean curvature of sphere in world units
    mean = 1.0/r;
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "radius in world units = " << r << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << mean << "  " << cm << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(), percentage*mean);

    // Test area, volume, average mean curvature and average gaussian curvature of sphere in world units
    gauss = 1.0/(r*r);
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "radius in world units = " << r << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << mean << "  " << cm << std::endl;
    //std::cerr << "Avg gaussian curvature of sphere = " << gauss << "  " << cg << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,  m.area(), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(), percentage*mean);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(gauss, m.avgGaussianCurvature(), percentage*gauss);
    CPPUNIT_ASSERT_EQUAL(0, m.genus());

    // Test measures of sphere in voxel units
    r /= voxelSize;
    area = 4*Pi*r*r;
    volume = 4.0/3.0*Pi*r*r*r;
    mean = 1.0/r;
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << curv << "  " << cm << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(false), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(false), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(false), percentage*mean);

    gauss = 1.0/(r*r);
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "radius in voxel units = " << r << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << mean << "  " << cm << std::endl;
    //std::cerr << "Avg gaussian curvature of sphere = " << gauss << "  " << cg << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(false), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(false), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(false), percentage*mean);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(gauss,  m.avgGaussianCurvature(false), percentage*gauss);
    CPPUNIT_ASSERT_EQUAL(0, m.genus());

    // Second sphere
    C = openvdb::Vec3f(5.4f, 6.4f, 8.4f);
    r = 0.57;
    sphere = openvdb::agents::createLevelSetSphere<GridT>(float(r), C, float(voxelSize));
    m.init(*sphere);

    // Test all measures of sphere in world units
    area = 4*Pi*r*r;
    volume = 4.0/3.0*Pi*r*r*r;
    mean = 1.0/r;
    gauss = 1.0/(r*r);
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "radius in world units = " << r << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << mean << "  " << cm << std::endl;
    //std::cerr << "Avg gaussian curvature of sphere = " << gauss << "  " << cg << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(), percentage*mean);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(gauss,  m.avgGaussianCurvature(), percentage*gauss);
    CPPUNIT_ASSERT_EQUAL(0, m.genus());
    //CPPUNIT_ASSERT_DOUBLES_EQUAL(area,  openvdb::agents::levelSetArea(*sphere),  percentage*area);
    //CPPUNIT_ASSERT_DOUBLES_EQUAL(volume,openvdb::agents::levelSetVolume(*sphere),percentage*volume);
    //CPPUNIT_ASSERT_EQUAL(0, openvdb::agents::levelSetGenus(*sphere));

     // Test all measures of sphere in voxel units
    r /= voxelSize;
    area = 4*Pi*r*r;
    volume = 4.0/3.0*Pi*r*r*r;
    mean = 1.0/r;
    gauss = 1.0/(r*r);
    //std::cerr << "\nArea of sphere = " << area << "  " << a << std::endl;
    //std::cerr << "Volume of sphere = " << volume << "  " << v << std::endl;
    //std::cerr << "radius in voxel units = " << r << std::endl;
    //std::cerr << "Avg mean curvature of sphere = " << mean << "  " << cm << std::endl;
    //std::cerr << "Avg gaussian curvature of sphere = " << gauss << "  " << cg << std::endl;
    // Test accuracy of computed measures to within 0.1% of the exact measure.
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,   m.area(false), percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume, m.volume(false), percentage*volume);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(mean,   m.avgMeanCurvature(false), percentage*mean);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(gauss,  m.avgGaussianCurvature(false), percentage*gauss);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(area,  openvdb::agents::levelSetArea(*sphere,false),
                                 percentage*area);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(volume,openvdb::agents::levelSetVolume(*sphere,false),
                                 percentage*volume);
    CPPUNIT_ASSERT_EQUAL(0, openvdb::agents::levelSetGenus(*sphere));

    // Read level set from file
    /*
    util::CpuTimer timer;
    openvdb::initialize();//required whenever I/O of OpenVDB files is performed!
    openvdb::io::File sourceFile("/usr/pic1/Data/OpenVDB/LevelSetModels/venusstatue.vdb");
    sourceFile.open();
    GridT::Ptr model = openvdb::gridPtrCast<GridT>(sourceFile.getGrids()->at(0));
    m.reinit(*model);

    //m.setGrainSize(1);
    timer.start("\nParallel measure of area and volume");
    m.measure(a, v, false);
    timer.stop();
    std::cerr << "Model: area = " << a << ", volume = " << v << std::endl;

    timer.start("\nParallel measure of area, volume and curvature");
    m.measure(a, v, c, false);
    timer.stop();
    std::cerr << "Model: area = " << a << ", volume = " << v
              << ", average curvature = " << c << std::endl;

    m.setGrainSize(0);
    timer.start("\nSerial measure of area and volume");
    m.measure(a, v, false);
    timer.stop();
    std::cerr << "Model: area = " << a << ", volume = " << v << std::endl;

    timer.start("\nSerial measure of area, volume and curvature");
    m.measure(a, v, c, false);
    timer.stop();
    std::cerr << "Model: area = " << a << ", volume = " << v
              << ", average curvature = " << c << std::endl;
    */

   {// testing total genus of multiple disjoint level set spheres with different radius
     const float dx = 0.5f, r = 50.0f;
     auto grid = openvdb::createLevelSet<openvdb::FloatGrid>(dx);
     CPPUNIT_ASSERT_THROW(openvdb::agents::levelSetGenus(*grid), openvdb::RuntimeError);
     for (int i=1; i<=3; ++i) {
       auto sphere = openvdb::agents::createLevelSetSphere<GridT>(r+float(i)*5.0f , openvdb::Vec3f(100.0f*float(i)), dx);
       openvdb::agents::csgUnion(*grid, *sphere);
       const int x = openvdb::agents::levelSetEulerCharacteristic(*grid);// since they are not overlapping re-normalization is not required
       //std::cerr << "Euler characteristics of " << i << " sphere(s) = " << x << std::endl;
       CPPUNIT_ASSERT_EQUAL(2*i, x);
     }
   }
   {// testing total genus of multiple disjoint level set cubes of different size
     const float dx = 0.5f, size = 50.0f;
     auto grid = openvdb::createLevelSet<openvdb::FloatGrid>(dx);
     CPPUNIT_ASSERT_THROW(openvdb::agents::levelSetGenus(*grid), openvdb::RuntimeError);
     for (int i=1; i<=2; ++i) {
       auto shape = openvdb::agents::createLevelSetCube<openvdb::FloatGrid>(size, openvdb::Vec3f(100.0f*float(i)), dx);
       openvdb::agents::csgUnion(*grid, *shape);
       const int x = openvdb::agents::levelSetEulerCharacteristic(*grid);
       //std::cerr << "Euler characteristics of " << i << " cubes(s) = " << x << std::endl;
       CPPUNIT_ASSERT_EQUAL(2*i, x);
     }
   }
   {// testing Euler characteristic and total genus of multiple intersecting (connected) level set spheres
     const float dx = 0.5f, r = 50.0f;
     auto grid = openvdb::createLevelSet<openvdb::FloatGrid>(dx);
     CPPUNIT_ASSERT_THROW(openvdb::agents::levelSetGenus(*grid), openvdb::RuntimeError);
     for (int i=1; i<=4; ++i) {
       auto sphere = openvdb::agents::createLevelSetSphere<GridT>( r , openvdb::Vec3f(30.0f*float(i), 0.0f, 0.0f), dx);
       openvdb::agents::csgUnion(*grid, *sphere);
       const int genus = openvdb::agents::levelSetGenus(*grid);
       const int x = openvdb::agents::levelSetEulerCharacteristic(*grid);
       //std::cerr << "Genus of " << i << " sphere(s) = " << genus << std::endl;
       CPPUNIT_ASSERT_EQUAL(0, genus);
       //std::cerr << "Euler characteristics of " << i << " sphere(s) = " << genus << std::endl;
       CPPUNIT_ASSERT_EQUAL(2, x);
     }
   }

}//testLevelSetMeasure

void
TestAgents::testMagnitude()
{
    using namespace openvdb;
    {
        FloatGrid::Ptr grid = FloatGrid::create(/*background=*/5.0);
        FloatTree& tree = grid->tree();
        CPPUNIT_ASSERT(tree.empty());

        const Coord dim(64,64,64);
        const Vec3f center(35.0f, 30.0f, 40.0f);
        const float radius=0.0f;
        unittest_util::makeSphere(dim, center, radius, *grid, unittest_util::SPHERE_DENSE);

        CPPUNIT_ASSERT(!tree.empty());
        CPPUNIT_ASSERT_EQUAL(dim[0]*dim[1]*dim[2], int(tree.activeVoxelCount()));

        VectorGrid::Ptr gradGrid = agents::gradient(*grid);
        CPPUNIT_ASSERT_EQUAL(int(tree.activeVoxelCount()), int(gradGrid->activeVoxelCount()));

        FloatGrid::Ptr mag = agents::magnitude(*gradGrid);
        CPPUNIT_ASSERT_EQUAL(int(tree.activeVoxelCount()), int(mag->activeVoxelCount()));

        FloatGrid::ConstAccessor accessor = mag->getConstAccessor();

        Coord xyz(35,30,30);
        float v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, v, 0.01);

        xyz.reset(35,10,40);
        v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, v, 0.01);
    }
    {
        // Test on a grid with (only) tile values.

        Vec3fGrid grid;
        Vec3fTree& tree = grid.tree();
        CPPUNIT_ASSERT(tree.empty());

        const Vec3f v(1.f, 2.f, 2.f);
        const float expectedLength = v.length();

        tree.addTile(/*level=*/1, Coord(-100), v, /*active=*/true);
        tree.addTile(/*level=*/1, Coord(100), v, /*active=*/true);

        CPPUNIT_ASSERT(!tree.empty());

        FloatGrid::Ptr length = agents::magnitude(grid);

        CPPUNIT_ASSERT_EQUAL(int(tree.activeVoxelCount()), int(length->activeVoxelCount()));

        for (auto it = length->cbeginValueOn(); it; ++it) {
            CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedLength, *it, 1.0e-6);
        }
    }
}


void
TestAgents::testMaskedMagnitude()
{
    using namespace openvdb;
    {
        FloatGrid::Ptr grid = FloatGrid::create(/*background=*/5.0);
        FloatTree& tree = grid->tree();
        CPPUNIT_ASSERT(tree.empty());

        const Coord dim(64,64,64);
        const Vec3f center(35.0f, 30.0f, 40.0f);
        const float radius=0.0f;
        unittest_util::makeSphere(dim, center, radius, *grid, unittest_util::SPHERE_DENSE);

        CPPUNIT_ASSERT(!tree.empty());
        CPPUNIT_ASSERT_EQUAL(dim[0]*dim[1]*dim[2], int(tree.activeVoxelCount()));

        VectorGrid::Ptr gradGrid = agents::gradient(*grid);
        CPPUNIT_ASSERT_EQUAL(int(tree.activeVoxelCount()), int(gradGrid->activeVoxelCount()));

        // create a masking grid
        const CoordBBox maskbbox(Coord(35, 30, 30), Coord(41, 41, 41));
        BoolGrid::Ptr maskGrid = BoolGrid::create(false);
        maskGrid->fill(maskbbox, true/*value*/, true/*activate*/);

        // compute the magnitude in masked region
        FloatGrid::Ptr mag = agents::magnitude(*gradGrid, *maskGrid);

        FloatGrid::ConstAccessor accessor = mag->getConstAccessor();

        // test in the masked region
        Coord xyz(35,30,30);
        CPPUNIT_ASSERT(maskbbox.isInside(xyz));
        float v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, v, 0.01);

        // test outside the masked region
        xyz.reset(35,10,40);
        CPPUNIT_ASSERT(!maskbbox.isInside(xyz));
        v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, v, 0.01);
    }
    {
        // Test on a grid with (only) tile values.

        Vec3fGrid grid;
        Vec3fTree& tree = grid.tree();
        CPPUNIT_ASSERT(tree.empty());

        const Vec3f v(1.f, 2.f, 2.f);
        const float expectedLength = v.length();

        tree.addTile(/*level=*/1, Coord(100), v, /*active=*/true);
        const int expectedActiveVoxelCount = int(tree.activeVoxelCount());
        tree.addTile(/*level=*/1, Coord(-100), v, /*active=*/true);

        CPPUNIT_ASSERT(!tree.empty());

        BoolGrid mask;
        mask.fill(CoordBBox(Coord(90), Coord(200)), true, true);

        FloatGrid::Ptr length = agents::magnitude(grid, mask);

        CPPUNIT_ASSERT_EQUAL(expectedActiveVoxelCount, int(length->activeVoxelCount()));

        for (auto it = length->cbeginValueOn(); it; ++it) {
            CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedLength, *it, 1.0e-6);
        }
    }
}


void
TestAgents::testNormalize()
{
    openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create(5.0);
    openvdb::FloatTree& tree = grid->tree();

    const openvdb::Coord dim(64,64,64);
    const openvdb::Vec3f center(35.0f, 30.0f, 40.0f);
    const float radius=10.0f;
    unittest_util::makeSphere<openvdb::FloatGrid>(
        dim,center,radius,*grid, unittest_util::SPHERE_DENSE);

    CPPUNIT_ASSERT_EQUAL(dim[0]*dim[1]*dim[2], int(tree.activeVoxelCount()));
    openvdb::Coord xyz(10, 20, 30);

    openvdb::VectorGrid::Ptr grad = openvdb::agents::gradient(*grid);

    using Vec3Type = openvdb::VectorGrid::ValueType;

    using ValueIter = openvdb::VectorGrid::ValueOnIter;

    struct Local {
        static inline Vec3Type op(const Vec3Type &x) { return x * 2.0f; }
        static inline void visit(const ValueIter& it) { it.setValue(op(*it)); }
    };

    openvdb::agents::foreach(grad->beginValueOn(), Local::visit, true);

    openvdb::VectorGrid::ConstAccessor accessor = grad->getConstAccessor();

    xyz = openvdb::Coord(35,10,40);
    Vec3Type v = accessor.getValue(xyz);
    //std::cerr << "\nPassed testNormalize(" << xyz << ")=" << v.length() << std::endl;
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0,v.length(),0.001);
    openvdb::VectorGrid::Ptr norm = openvdb::agents::normalize(*grad);

    accessor = norm->getConstAccessor();
    v = accessor.getValue(xyz);
    //std::cerr << "\nPassed testNormalize(" << xyz << ")=" << v.length() << std::endl;
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, v.length(), 0.0001);
}


void
TestAgents::testMaskedNormalize()
{
    openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create(5.0);
    openvdb::FloatTree& tree = grid->tree();

    const openvdb::Coord dim(64,64,64);
    const openvdb::Vec3f center(35.0f, 30.0f, 40.0f);
    const float radius=10.0f;
    unittest_util::makeSphere<openvdb::FloatGrid>(
        dim,center,radius,*grid, unittest_util::SPHERE_DENSE);

    CPPUNIT_ASSERT_EQUAL(dim[0]*dim[1]*dim[2], int(tree.activeVoxelCount()));
    openvdb::Coord xyz(10, 20, 30);

    openvdb::VectorGrid::Ptr grad = openvdb::agents::gradient(*grid);

    using Vec3Type = openvdb::VectorGrid::ValueType;

    using ValueIter = openvdb::VectorGrid::ValueOnIter;

    struct Local {
        static inline Vec3Type op(const Vec3Type &x) { return x * 2.0f; }
        static inline void visit(const ValueIter& it) { it.setValue(op(*it)); }
    };

    openvdb::agents::foreach(grad->beginValueOn(), Local::visit, true);

    openvdb::VectorGrid::ConstAccessor accessor = grad->getConstAccessor();

    xyz = openvdb::Coord(35,10,40);
    Vec3Type v = accessor.getValue(xyz);

    // create a masking grid

    const openvdb::CoordBBox maskbbox(openvdb::Coord(35, 30, 30), openvdb::Coord(41, 41, 41));
    openvdb::BoolGrid::Ptr maskGrid = openvdb::BoolGrid::create(false);
    maskGrid->fill(maskbbox, true/*value*/, true/*activate*/);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0,v.length(),0.001);

    // compute the normalized valued in the masked region
    openvdb::VectorGrid::Ptr norm = openvdb::agents::normalize(*grad, *maskGrid);

    accessor = norm->getConstAccessor();
    { // outside the masked region
        CPPUNIT_ASSERT(!maskbbox.isInside(xyz));
        v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, v.length(), 0.0001);
    }
    { // inside the masked region
        xyz.reset(35, 30, 30);
        v = accessor.getValue(xyz);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, v.length(), 0.0001);
    }
}

////////////////////////////////////////


void
TestAgents::testPointAdvect()
{
    {
        // Setup:    Advect a number of points in a uniform velocity field (1,1,1).
        //           over a time dt=1 with each of the 4 different advection schemes.
        //           Points initialized at latice points.
        //
        // Uses:     FloatTree (velocity), collocated sampling, advection
        //
        // Expected: All advection schemes will have the same result.  Each point will
        //           be advanced to a new latice point.  The i-th point will be at (i+1,i+1,i+1)
        //

        const size_t numPoints = 2000000;

        // create a uniform velocity field in SINGLE PRECISION
        const openvdb::Vec3f velocityBackground(1, 1, 1);
        openvdb::Vec3fGrid::Ptr velocityGrid = openvdb::Vec3fGrid::create(velocityBackground);

        // using all the default template arguments
        openvdb::agents::PointAdvect<> advectionAgent(*velocityGrid);

        // create points
        std::vector<openvdb::Vec3f> pointList(numPoints);  /// larger than the tbb chunk size
        for (size_t i = 0; i < numPoints; i++) {
            pointList[i] = openvdb::Vec3f(float(i), float(i), float(i));
        }

        for (unsigned int order = 1; order < 5; ++order) {
            // check all four time integrations schemes
            // construct an advection agent.  By default the number of cpt iterations is zero
            advectionAgent.setIntegrationOrder(order);
            advectionAgent.advect(pointList, /*dt=*/1.0, /*iterations=*/1);

            // check locations
            for (size_t i = 0; i < numPoints; i++) {
                openvdb::Vec3f expected(float(i + 1), float(i + 1), float(i + 1));
                CPPUNIT_ASSERT_EQUAL(expected, pointList[i]);
            }
            // reset values
            for (size_t i = 0; i < numPoints; i++) {
                pointList[i] = openvdb::Vec3f(float(i), float(i), float(i));
            }
        }

    }

    {
        // Setup:    Advect a number of points in a uniform velocity field (1,1,1).
        //           over a time dt=1 with each of the 4 different advection schemes.
        //           And then project the point location onto the x-y plane
        //           Points initialized at latice points.
        //
        // Uses:     DoubleTree (velocity), staggered sampling, constraint projection, advection
        //
        // Expected: All advection schemes will have the same result.  Modes 1-4: Each point will
        //           be advanced to a new latice point and projected to x-y plane.
        //           The i-th point will be at (i+1,i+1,0).  For mode 0 (no advection), i-th point
        //           will be found at (i, i, 0)

        const size_t numPoints = 4;

        // create a uniform velocity field in DOUBLE PRECISION
        const openvdb::Vec3d velocityBackground(1, 1, 1);
        openvdb::Vec3dGrid::Ptr velocityGrid = openvdb::Vec3dGrid::create(velocityBackground);

        // create a simple (horizontal) constraint field valid for a
        // (-10,10)x(-10,10)x(-10,10)
        const openvdb::Vec3d cptBackground(0, 0, 0);
        openvdb::Vec3dGrid::Ptr cptGrid = openvdb::Vec3dGrid::create(cptBackground);
        openvdb::Vec3dTree& cptTree = cptGrid->tree();

        // create points
        std::vector<openvdb::Vec3d> pointList(numPoints);
        for (unsigned int i = 0; i < numPoints; i++) pointList[i] = openvdb::Vec3d(i, i, i);

        // Initialize the constraint field in a [-10,10]x[-10,10]x[-10,10] box
        // this test will only work if the points remain in the box
        openvdb::Coord ijk(0, 0, 0);
        for (int i = -10; i < 11; i++) {
            ijk.setX(i);
            for (int j = -10; j < 11; j++) {
                ijk.setY(j);
                for (int k = -10; k < 11; k++) {
                    ijk.setZ(k);
                    // set the value as projection onto the x-y plane
                    cptTree.setValue(ijk, openvdb::Vec3d(i, j, 0));
                }
            }
        }

        // construct an advection agent.  By default the number of cpt iterations is zero
        openvdb::agents::ConstrainedPointAdvect<openvdb::Vec3dGrid,
            std::vector<openvdb::Vec3d>, true> constrainedAdvectionAgent(*velocityGrid, *cptGrid, 0);
        constrainedAdvectionAgent.setThreaded(false);

        // change the number of constraint interation from default 0 to 5
        constrainedAdvectionAgent.setConstraintIterations(5);

        // test the pure-projection mode (order = 0)
        constrainedAdvectionAgent.setIntegrationOrder(0);

        // change the number of constraint interation (from 0 to 5)
        constrainedAdvectionAgent.setConstraintIterations(5);

        constrainedAdvectionAgent.advect(pointList, /*dt=*/1.0, /*iterations=*/1);

        // check locations
        for (unsigned int i = 0; i < numPoints; i++) {
            openvdb::Vec3d expected(i, i, 0);  // location (i, i, i) projected on to x-y plane
            for (int n=0; n<3; ++n) {
                CPPUNIT_ASSERT_DOUBLES_EQUAL(expected[n], pointList[i][n], /*tolerance=*/1e-6);
            }
        }

        // reset values
        for (unsigned int i = 0; i < numPoints; i++) pointList[i] = openvdb::Vec3d(i, i, i);

        // test all four time integrations schemes
        for (unsigned int order = 1; order < 5; ++order) {

            constrainedAdvectionAgent.setIntegrationOrder(order);

            constrainedAdvectionAgent.advect(pointList, /*dt=*/1.0, /*iterations=*/1);

            // check locations
            for (unsigned int i = 0; i < numPoints; i++) {
                openvdb::Vec3d expected(i+1, i+1, 0); // location (i,i,i) projected onto x-y plane
                for (int n=0; n<3; ++n) {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(expected[n], pointList[i][n], /*tolerance=*/1e-6);
                }
            }
            // reset values
            for (unsigned int i = 0; i < numPoints; i++) pointList[i] = openvdb::Vec3d(i, i, i);
        }
    }
}


////////////////////////////////////////


namespace {

    struct PointList
    {
        struct Point { double x,y,z; };
        std::vector<Point> list;
        openvdb::Index64 size() const { return openvdb::Index64(list.size()); }
        void add(const openvdb::Vec3d &p) { Point q={p[0],p[1],p[2]}; list.push_back(q); }
    };
}


void
TestAgents::testPointScatter()
{
    using GridType = openvdb::FloatGrid;
    const openvdb::Coord dim(64, 64, 64);
    const openvdb::Vec3f center(35.0f, 30.0f, 40.0f);
    const float radius = 20.0;
    using RandGen = std::mersenne_twister_engine<uint32_t, 32, 351, 175, 19,
        0xccab8ee7, 11, 0xffffffff, 7, 0x31b6ab00, 15, 0xffe50000, 17, 1812433253>; // mt11213b
    RandGen mtRand;

    GridType::Ptr grid = GridType::create(/*background=*/2.0);
    unittest_util::makeSphere<GridType>(
        dim, center, radius, *grid, unittest_util::SPHERE_DENSE_NARROW_BAND);

    {// test fixed point count scattering
        const openvdb::Index64 pointCount = 1000;
        PointList points;
        openvdb::agents::UniformPointScatter<PointList, RandGen> scatter(points, pointCount, mtRand);
        scatter.operator()<GridType>(*grid);
        CPPUNIT_ASSERT_EQUAL( pointCount, scatter.getPointCount() );
        CPPUNIT_ASSERT_EQUAL( pointCount, points.size() );
    }
    {// test uniform density scattering
        const float density = 1.0f;//per volume = per voxel since voxel size = 1
        PointList points;
        openvdb::agents::UniformPointScatter<PointList, RandGen> scatter(points, density, mtRand);
        scatter.operator()<GridType>(*grid);
        CPPUNIT_ASSERT_EQUAL( scatter.getVoxelCount(), scatter.getPointCount() );
        CPPUNIT_ASSERT_EQUAL( scatter.getVoxelCount(), points.size() );
    }
    {// test non-uniform density scattering
        const float density = 1.0f;//per volume = per voxel since voxel size = 1
        PointList points;
        openvdb::agents::NonUniformPointScatter<PointList, RandGen> scatter(points, density, mtRand);
        scatter.operator()<GridType>(*grid);
        CPPUNIT_ASSERT( scatter.getVoxelCount() < scatter.getPointCount() );
        CPPUNIT_ASSERT_EQUAL( scatter.getPointCount(), points.size() );
    }
     {// test dense uniform scattering
        const size_t pointsPerVoxel = 8;
        PointList points;
        openvdb::agents::DenseUniformPointScatter<PointList, RandGen>
            scatter(points, pointsPerVoxel, mtRand);
        scatter.operator()<GridType>(*grid);
        CPPUNIT_ASSERT_EQUAL( scatter.getVoxelCount()*pointsPerVoxel, scatter.getPointCount() );
        CPPUNIT_ASSERT_EQUAL( scatter.getPointCount(), points.size() );
    }
}

////////////////////////////////////////

void
TestAgents::testVolumeAdvect()
{
    using namespace openvdb;

    Vec3fGrid velocity(Vec3f(1.0f, 0.0f, 0.0f));
    using GridT = FloatGrid;
    using AdvT = agents::VolumeAdvection<Vec3fGrid>;
    using SamplerT = agents::Sampler<1>;

    {//test non-uniform grids (throws)
        GridT::Ptr density0 = GridT::create(0.0f);
        density0->transform().preScale(Vec3d(1.0, 2.0, 3.0));//i.e. non-uniform voxels
        AdvT a(velocity);
        CPPUNIT_ASSERT_THROW((a.advect<GridT, SamplerT>(*density0, 0.1f)), RuntimeError);
    }

    {// test spatialOrder and temporalOrder
        AdvT a(velocity);

        // Default should be SEMI
        CPPUNIT_ASSERT_EQUAL(1, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(1, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());

        a.setIntegrator(agents::Scheme::SEMI);
        CPPUNIT_ASSERT_EQUAL(1, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(1, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());

        a.setIntegrator(agents::Scheme::MID);
        CPPUNIT_ASSERT_EQUAL(1, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(2, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());

        a.setIntegrator(agents::Scheme::RK3);
        CPPUNIT_ASSERT_EQUAL(1, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(3, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());

        a.setIntegrator(agents::Scheme::RK4);
        CPPUNIT_ASSERT_EQUAL(1, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(4, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());

        a.setIntegrator(agents::Scheme::MAC);
        CPPUNIT_ASSERT_EQUAL(2, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(2, a.temporalOrder());
        CPPUNIT_ASSERT( a.isLimiterOn());

        a.setIntegrator(agents::Scheme::BFECC);
        CPPUNIT_ASSERT_EQUAL(2, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(2, a.temporalOrder());
        CPPUNIT_ASSERT( a.isLimiterOn());

        a.setLimiter(agents::Scheme::NO_LIMITER);
        CPPUNIT_ASSERT_EQUAL(2, a.spatialOrder());
        CPPUNIT_ASSERT_EQUAL(2, a.temporalOrder());
        CPPUNIT_ASSERT(!a.isLimiterOn());
    }

    {//test RK4 advect without a mask
        GridT::Ptr density0 = GridT::create(0.0f), density1;
        density0->fill(CoordBBox(Coord(0),Coord(6)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord( 3,3,3)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(24,3,3)), 0.0f);
        CPPUNIT_ASSERT( density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT(!density0->tree().isValueOn(Coord(24,3,3)));

        AdvT a(velocity);
        a.setIntegrator(agents::Scheme::RK4);
        for (int i=1; i<=240; ++i) {
            density1 = a.advect<GridT, SamplerT>(*density0, 0.1f);
            //std::ostringstream ostr;
            //ostr << "densityAdvect" << "_" << i << ".vdb";
            //std::cerr << "Writing " << ostr.str() << std::endl;
            //openvdb::io::File file(ostr.str());
            //openvdb::GridPtrVec grids;
            //grids.push_back(density1);
            //file.write(grids);
            density0 = density1;
        }
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(3,3,3)), 0.0f);
        CPPUNIT_ASSERT(density0->tree().getValue(Coord(24,3,3)) > 0.0f);
        CPPUNIT_ASSERT(!density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT( density0->tree().isValueOn(Coord(24,3,3)));
    }
    {//test MAC advect without a mask
        GridT::Ptr density0 = GridT::create(0.0f), density1;
        density0->fill(CoordBBox(Coord(0),Coord(6)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord( 3,3,3)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(24,3,3)), 0.0f);
        CPPUNIT_ASSERT( density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT(!density0->tree().isValueOn(Coord(24,3,3)));

        AdvT a(velocity);
        a.setIntegrator(agents::Scheme::BFECC);
        for (int i=1; i<=240; ++i) {
            density1 = a.advect<GridT, SamplerT>(*density0, 0.1f);
            //std::ostringstream ostr;
            //ostr << "densityAdvect" << "_" << i << ".vdb";
            //std::cerr << "Writing " << ostr.str() << std::endl;
            //openvdb::io::File file(ostr.str());
            //openvdb::GridPtrVec grids;
            //grids.push_back(density1);
            //file.write(grids);
            density0 = density1;
        }
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(3,3,3)), 0.0f);
        CPPUNIT_ASSERT(density0->tree().getValue(Coord(24,3,3)) > 0.0f);
        CPPUNIT_ASSERT(!density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT( density0->tree().isValueOn(Coord(24,3,3)));
    }
    {//test advect with a mask
        GridT::Ptr density0 = GridT::create(0.0f), density1;
        density0->fill(CoordBBox(Coord(0),Coord(6)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord( 3,3,3)), 1.0f);
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(24,3,3)), 0.0f);
        CPPUNIT_ASSERT( density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT(!density0->tree().isValueOn(Coord(24,3,3)));

        BoolGrid::Ptr mask = BoolGrid::create(false);
        mask->fill(CoordBBox(Coord(4,0,0),Coord(30,8,8)), true);

        AdvT a(velocity);
        a.setGrainSize(0);
        a.setIntegrator(agents::Scheme::MAC);
        //a.setIntegrator(agents::Scheme::BFECC);
        //a.setIntegrator(agents::Scheme::RK4);
        for (int i=1; i<=240; ++i) {
            density1 = a.advect<GridT, BoolGrid, SamplerT>(*density0, *mask, 0.1f);
            //std::ostringstream ostr;
            //ostr << "densityAdvectMask" << "_" << i << ".vdb";
            //std::cerr << "Writing " << ostr.str() << std::endl;
            //openvdb::io::File file(ostr.str());
            //openvdb::GridPtrVec grids;
            //grids.push_back(density1);
            //file.write(grids);
            density0 = density1;
        }
        CPPUNIT_ASSERT_EQUAL(density0->tree().getValue(Coord(3,3,3)), 1.0f);
        CPPUNIT_ASSERT(density0->tree().getValue(Coord(24,3,3)) > 0.0f);
        CPPUNIT_ASSERT(density0->tree().isValueOn(Coord( 3,3,3)));
        CPPUNIT_ASSERT(density0->tree().isValueOn(Coord(24,3,3)));
    }

    /*
    {//benchmark on a sphere
        util::CpuTimer timer;
        GridT::Ptr density0 = GridT::create(0.0f), density1;
        density0->fill(CoordBBox(Coord(0), Coord(600)), 1.0f);
        timer.start("densify");
        density0->tree().voxelizeActiveTiles();
        timer.stop();
        AdvT a(velocity);
        a.setGrainSize(1);
        //a.setLimiter(agents::Scheme::NO_LIMITER);
        //a.setIntegrator(agents::Scheme::MAC);
        //a.setIntegrator(agents::Scheme::BFECC);
        a.setIntegrator(agents::Scheme::RK4);

        for (int i=1; i<=10; ++i) {
            timer.start("Volume Advection");
            density1 = a.advect<GridT, SamplerT>(*density0, 0.1f);
            timer.stop();
            std::ostringstream ostr;
            ostr << "densityAdvectMask" << "_" << i << ".vdb";
            std::cerr << "Writing " << ostr.str() << std::endl;
            io::File file(ostr.str());
            GridPtrVec grids;
            grids.push_back(density1);
            file.write(grids);
            density0.swap(density1);
            }
            }
    */
}// testVolumeAdvect

////////////////////////////////////////


void
TestAgents::testFloatApply()
{
    using ValueIter = openvdb::FloatTree::ValueOnIter;

    struct Local {
        static inline float op(float x) { return x * 2.f; }
        static inline void visit(const ValueIter& it) { it.setValue(op(*it)); }
    };

    const float background = 1.0;
    openvdb::FloatTree tree(background);

    const int MIN = -1000, MAX = 1000, STEP = 50;
    openvdb::Coord xyz;
    for (int z = MIN; z < MAX; z += STEP) {
        xyz.setZ(z);
        for (int y = MIN; y < MAX; y += STEP) {
            xyz.setY(y);
            for (int x = MIN; x < MAX; x += STEP) {
                xyz.setX(x);
                tree.setValue(xyz, float(x + y + z));
            }
        }
    }
    /// @todo set some tile values

    openvdb::agents::foreach(tree.begin<ValueIter>(), Local::visit, /*threaded=*/true);

    float expected = Local::op(background);
    //CPPUNIT_ASSERT_DOUBLES_EQUAL(expected, tree.background(), /*tolerance=*/0.0);
    //expected = Local::op(-background);
    //CPPUNIT_ASSERT_DOUBLES_EQUAL(expected, -tree.background(), /*tolerance=*/0.0);

    for (openvdb::FloatTree::ValueOnCIter it = tree.cbeginValueOn(); it; ++it) {
        xyz = it.getCoord();
        expected = Local::op(float(xyz[0] + xyz[1] + xyz[2]));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(expected, it.getValue(), /*tolerance=*/0.0);
    }
}


////////////////////////////////////////


namespace {

template<typename IterT>
struct MatMul {
    openvdb::math::Mat3s mat;
    MatMul(const openvdb::math::Mat3s& _mat): mat(_mat) {}
    openvdb::Vec3s xform(const openvdb::Vec3s& v) const { return mat.transform(v); }
    void operator()(const IterT& it) const { it.setValue(xform(*it)); }
};

}


void
TestAgents::testVectorApply()
{
    using ValueIter = openvdb::VectorTree::ValueOnIter;

    const openvdb::Vec3s background(1, 1, 1);
    openvdb::VectorTree tree(background);

    const int MIN = -1000, MAX = 1000, STEP = 80;
    openvdb::Coord xyz;
    for (int z = MIN; z < MAX; z += STEP) {
        xyz.setZ(z);
        for (int y = MIN; y < MAX; y += STEP) {
            xyz.setY(y);
            for (int x = MIN; x < MAX; x += STEP) {
                xyz.setX(x);
                tree.setValue(xyz, openvdb::Vec3s(float(x), float(y), float(z)));
            }
        }
    }
    /// @todo set some tile values

    MatMul<ValueIter> op(openvdb::math::Mat3s(1, 2, 3, -1, -2, -3, 3, 2, 1));
    openvdb::agents::foreach(tree.beginValueOn(), op, /*threaded=*/true);

    openvdb::Vec3s expected;
    for (openvdb::VectorTree::ValueOnCIter it = tree.cbeginValueOn(); it; ++it) {
        xyz = it.getCoord();
        expected = op.xform(openvdb::Vec3s(float(xyz[0]), float(xyz[1]), float(xyz[2])));
        CPPUNIT_ASSERT_EQUAL(expected, it.getValue());
    }
}


////////////////////////////////////////


namespace {

struct AccumSum {
    int64_t sum; int joins;
    AccumSum(): sum(0), joins(0) {}
    void operator()(const openvdb::Int32Tree::ValueOnCIter& it)
    {
        if (it.isVoxelValue()) sum += *it;
        else sum += (*it) * it.getVoxelCount();
    }
    void join(AccumSum& other) { sum += other.sum; joins += 1 + other.joins; }
};


struct AccumLeafVoxelCount {
    using LeafRange = openvdb::tree::LeafManager<openvdb::Int32Tree>::LeafRange;
    openvdb::Index64 count;
    AccumLeafVoxelCount(): count(0) {}
    void operator()(const LeafRange::Iterator& it) { count += it->onVoxelCount(); }
    void join(AccumLeafVoxelCount& other) { count += other.count; }
};

}


void
TestAgents::testAccumulate()
{
    using namespace openvdb;

    const int value = 2;
    Int32Tree tree(/*background=*/0);
    tree.fill(CoordBBox::createCube(Coord(0), 198), value, /*active=*/true);

    const int64_t expected = tree.activeVoxelCount() * value;
    {
        AccumSum op;
        agents::accumulate(tree.cbeginValueOn(), op, /*threaded=*/false);
        CPPUNIT_ASSERT_EQUAL(expected, op.sum);
        CPPUNIT_ASSERT_EQUAL(0, op.joins);
    }
    {
        AccumSum op;
        agents::accumulate(tree.cbeginValueOn(), op, /*threaded=*/true);
        CPPUNIT_ASSERT_EQUAL(expected, op.sum);
    }
    {
        AccumLeafVoxelCount op;
        tree::LeafManager<Int32Tree> mgr(tree);
        agents::accumulate(mgr.leafRange().begin(), op, /*threaded=*/true);
        CPPUNIT_ASSERT_EQUAL(tree.activeLeafVoxelCount(), op.count);
    }
}


////////////////////////////////////////


namespace {

template<typename InIterT, typename OutTreeT>
struct FloatToVec
{
    using ValueT = typename InIterT::ValueT;
    using Accessor = typename openvdb::tree::ValueAccessor<OutTreeT>;

    // Transform a scalar value into a vector value.
    static openvdb::Vec3s toVec(const ValueT& v) { return openvdb::Vec3s(v, v*2, v*3); }

    FloatToVec() { numTiles = 0; }

    void operator()(const InIterT& it, Accessor& acc)
    {
        if (it.isVoxelValue()) { // set a single voxel
            acc.setValue(it.getCoord(), toVec(*it));
        } else { // fill an entire tile
            numTiles.fetch_and_increment();
            openvdb::CoordBBox bbox;
            it.getBoundingBox(bbox);
            acc.tree().fill(bbox, toVec(*it));
        }
    }

    tbb::atomic<int> numTiles;
};

}


void
TestAgents::testTransformValues()
{
    using openvdb::CoordBBox;
    using openvdb::Coord;
    using openvdb::Vec3s;

    using Tree323f = openvdb::tree::Tree4<float, 3, 2, 3>::Type;
    using Tree323v = openvdb::tree::Tree4<Vec3s, 3, 2, 3>::Type;

    const float background = 1.0;
    Tree323f ftree(background);

    const int MIN = -1000, MAX = 1000, STEP = 80;
    Coord xyz;
    for (int z = MIN; z < MAX; z += STEP) {
        xyz.setZ(z);
        for (int y = MIN; y < MAX; y += STEP) {
            xyz.setY(y);
            for (int x = MIN; x < MAX; x += STEP) {
                xyz.setX(x);
                ftree.setValue(xyz, float(x + y + z));
            }
        }
    }
    // Set some tile values.
    ftree.fill(CoordBBox(Coord(1024), Coord(1024 + 8 - 1)), 3 * 1024); // level-1 tile
    ftree.fill(CoordBBox(Coord(2048), Coord(2048 + 32 - 1)), 3 * 2048); // level-2 tile
    ftree.fill(CoordBBox(Coord(3072), Coord(3072 + 256 - 1)), 3 * 3072); // level-3 tile

    for (int shareOp = 0; shareOp <= 1; ++shareOp) {
        FloatToVec<Tree323f::ValueOnCIter, Tree323v> op;
        Tree323v vtree;
        openvdb::agents::transformValues(ftree.cbeginValueOn(), vtree, op,
            /*threaded=*/true, shareOp);

        // The tile count is accurate only if the functor is shared.  Otherwise,
        // it is initialized to zero in the main thread and never changed.
        CPPUNIT_ASSERT_EQUAL(shareOp ? 3 : 0, int(op.numTiles));

        Vec3s expected;
        for (Tree323v::ValueOnCIter it = vtree.cbeginValueOn(); it; ++it) {
            xyz = it.getCoord();
            expected = op.toVec(float(xyz[0] + xyz[1] + xyz[2]));
            CPPUNIT_ASSERT_EQUAL(expected, it.getValue());
        }
        // Check values inside the tiles.
        CPPUNIT_ASSERT_EQUAL(op.toVec(3 * 1024), vtree.getValue(Coord(1024 + 4)));
        CPPUNIT_ASSERT_EQUAL(op.toVec(3 * 2048), vtree.getValue(Coord(2048 + 16)));
        CPPUNIT_ASSERT_EQUAL(op.toVec(3 * 3072), vtree.getValue(Coord(3072 + 128)));
    }
}


////////////////////////////////////////


void
TestAgents::testUtil()
{
    using openvdb::CoordBBox;
    using openvdb::Coord;
    using openvdb::Vec3s;

    using CharTree = openvdb::tree::Tree4<bool, 3, 2, 3>::Type;

    // Test boolean operators
    CharTree treeA(false), treeB(false);

    treeA.fill(CoordBBox(Coord(-10), Coord(10)), true);
    treeA.voxelizeActiveTiles();

    treeB.fill(CoordBBox(Coord(-10), Coord(10)), true);
    treeB.voxelizeActiveTiles();

    const size_t voxelCountA = treeA.activeVoxelCount();
    const size_t voxelCountB = treeB.activeVoxelCount();

    CPPUNIT_ASSERT_EQUAL(voxelCountA, voxelCountB);

    CharTree::Ptr tree = openvdb::util::leafTopologyDifference(treeA, treeB);
    CPPUNIT_ASSERT(tree->activeVoxelCount() == 0);

    tree = openvdb::util::leafTopologyIntersection(treeA, treeB);
    CPPUNIT_ASSERT(tree->activeVoxelCount() == voxelCountA);

    treeA.fill(CoordBBox(Coord(-10), Coord(22)), true);
    treeA.voxelizeActiveTiles();

    const size_t voxelCount = treeA.activeVoxelCount();

    tree = openvdb::util::leafTopologyDifference(treeA, treeB);
    CPPUNIT_ASSERT(tree->activeVoxelCount() == (voxelCount - voxelCountA));

    tree = openvdb::util::leafTopologyIntersection(treeA, treeB);
    CPPUNIT_ASSERT(tree->activeVoxelCount() == voxelCountA);
}


////////////////////////////////////////


void
TestAgents::testVectorTransformer()
{
    using namespace openvdb;

    Mat4d xform = Mat4d::identity();
    xform.preTranslate(Vec3d(0.1, -2.5, 3));
    xform.preScale(Vec3d(0.5, 1.1, 2));
    xform.preRotate(math::X_AXIS, 30.0 * M_PI / 180.0);
    xform.preRotate(math::Y_AXIS, 300.0 * M_PI / 180.0);

    Mat4d invXform = xform.inverse();
    invXform = invXform.transpose();

    {
        // Set some vector values in a grid, then verify that agents::transformVectors()
        // transforms them as expected for each VecType.

        const Vec3s refVec0(0, 0, 0), refVec1(1, 0, 0), refVec2(0, 1, 0), refVec3(0, 0, 1);

        Vec3SGrid grid;
        Vec3SGrid::Accessor acc = grid.getAccessor();

#define resetGrid() \
    { \
        grid.clear(); \
        acc.setValue(Coord(0), refVec0); \
        acc.setValue(Coord(1), refVec1); \
        acc.setValue(Coord(2), refVec2); \
        acc.setValue(Coord(3), refVec3); \
    }

        // Verify that grid values are in world space by default.
        CPPUNIT_ASSERT(grid.isInWorldSpace());

        resetGrid();
        grid.setVectorType(VEC_INVARIANT);
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT(acc.getValue(Coord(0)).eq(refVec0));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(refVec1));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(refVec2));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(refVec3));

        resetGrid();
        grid.setVectorType(VEC_COVARIANT);
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT(acc.getValue(Coord(0)).eq(invXform.transform3x3(refVec0)));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(invXform.transform3x3(refVec1)));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(invXform.transform3x3(refVec2)));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(invXform.transform3x3(refVec3)));

        resetGrid();
        grid.setVectorType(VEC_COVARIANT_NORMALIZE);
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT_EQUAL(refVec0, acc.getValue(Coord(0)));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(invXform.transform3x3(refVec1).unit()));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(invXform.transform3x3(refVec2).unit()));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(invXform.transform3x3(refVec3).unit()));

        resetGrid();
        grid.setVectorType(VEC_CONTRAVARIANT_RELATIVE);
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT(acc.getValue(Coord(0)).eq(xform.transform3x3(refVec0)));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(xform.transform3x3(refVec1)));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(xform.transform3x3(refVec2)));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(xform.transform3x3(refVec3)));

        resetGrid();
        grid.setVectorType(VEC_CONTRAVARIANT_ABSOLUTE);
        /// @todo This doesn't really test the behavior w.r.t. homogeneous coords.
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT(acc.getValue(Coord(0)).eq(xform.transformH(refVec0)));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(xform.transformH(refVec1)));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(xform.transformH(refVec2)));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(xform.transformH(refVec3)));

        // Verify that transformVectors() has no effect on local-space grids.
        resetGrid();
        grid.setVectorType(VEC_CONTRAVARIANT_RELATIVE);
        grid.setIsInWorldSpace(false);
        agents::transformVectors(grid, xform);
        CPPUNIT_ASSERT(acc.getValue(Coord(0)).eq(refVec0));
        CPPUNIT_ASSERT(acc.getValue(Coord(1)).eq(refVec1));
        CPPUNIT_ASSERT(acc.getValue(Coord(2)).eq(refVec2));
        CPPUNIT_ASSERT(acc.getValue(Coord(3)).eq(refVec3));

#undef resetGrid
    }
    {
        // Verify that transformVectors() operates only on vector-valued grids.
        FloatGrid scalarGrid;
        CPPUNIT_ASSERT_THROW(agents::transformVectors(scalarGrid, xform), TypeError);
    }
}


////////////////////////////////////////


void
TestAgents::testPrune()
{
    /// @todo Add more unit-tests!

    using namespace openvdb;

    {// try prunning a tree with const values
        const float value = 5.345f;

        FloatTree tree(value);
        CPPUNIT_ASSERT_EQUAL(Index32(0), tree.leafCount());
        CPPUNIT_ASSERT_EQUAL(Index32(1), tree.nonLeafCount()); // root node
        CPPUNIT_ASSERT(tree.empty());

        tree.fill(CoordBBox(Coord(-10), Coord(10)), value, /*active=*/false);
        CPPUNIT_ASSERT(!tree.empty());

        agents::prune(tree);

        CPPUNIT_ASSERT_EQUAL(Index32(0), tree.leafCount());
        CPPUNIT_ASSERT_EQUAL(Index32(1), tree.nonLeafCount()); // root node
        CPPUNIT_ASSERT(tree.empty());
    }

    {// Prune a tree with a single leaf node with random values in the range [0,1]
        using LeafNodeT = tree::LeafNode<float,3>;
        const float val = 1.0, tol = 1.1f;

        // Fill a leaf node with random values in the range [0,1]
        LeafNodeT *leaf = new LeafNodeT(Coord(0), val, true);
        math::Random01 r(145);
        std::vector<float> data(LeafNodeT::NUM_VALUES);
        for (Index i=0; i<LeafNodeT::NUM_VALUES; ++i) {
            const float v = float(r());
            data[i] = v;
            leaf->setValueOnly(i, v);
        }

        // Insert leaf node into an empty tree
        FloatTree tree(val);
        tree.addLeaf(leaf);

        CPPUNIT_ASSERT_EQUAL(Index32(1), tree.leafCount());
        CPPUNIT_ASSERT_EQUAL(Index32(3), tree.nonLeafCount()); // root+2*internal

        agents::prune(tree);// tolerance is zero

        CPPUNIT_ASSERT_EQUAL(Index32(1), tree.leafCount());
        CPPUNIT_ASSERT_EQUAL(Index32(3), tree.nonLeafCount()); // root+2*internal

        agents::prune(tree, tol);

        CPPUNIT_ASSERT_EQUAL(Index32(0), tree.leafCount());
        CPPUNIT_ASSERT_EQUAL(Index32(3), tree.nonLeafCount()); // root+2*internal

        std::sort(data.begin(), data.end());
        const float median = data[(LeafNodeT::NUM_VALUES-1)>>1];

        ASSERT_DOUBLES_EXACTLY_EQUAL(median, tree.getValue(Coord(0)));
    }

    /*
    {// Benchmark serial prune
        util::CpuTimer timer;
        initialize();//required whenever I/O of OpenVDB files is performed!
        io::File sourceFile("/usr/pic1/Data/OpenVDB/LevelSetModels/crawler.vdb");
        sourceFile.open(false);//disable delayed loading
        FloatGrid::Ptr grid = gridPtrCast<FloatGrid>(sourceFile.getGrids()->at(0));
        const Index32 leafCount = grid->tree().leafCount();

        timer.start("\nSerial tolerance prune");
        grid->tree().prune();
        timer.stop();
        CPPUNIT_ASSERT_EQUAL(leafCount, grid->tree().leafCount());
    }
    {// Benchmark parallel prune
        util::CpuTimer timer;
        initialize();//required whenever I/O of OpenVDB files is performed!
        io::File sourceFile("/usr/pic1/Data/OpenVDB/LevelSetModels/crawler.vdb");
        sourceFile.open(false);//disable delayed loading
        FloatGrid::Ptr grid = gridPtrCast<FloatGrid>(sourceFile.getGrids()->at(0));
        const Index32 leafCount = grid->tree().leafCount();

        timer.start("\nParallel tolerance prune");
        agents::prune(grid->tree());
        timer.stop();
        CPPUNIT_ASSERT_EQUAL(leafCount, grid->tree().leafCount());
    }
    */
}
