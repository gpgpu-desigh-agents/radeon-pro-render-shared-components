// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#include <cppunit/extensions/HelperMacros.h>

#include <limits>
#include <openvdb/openvdb.h>
#include <openvdb/Exceptions.h>
#include <openvdb/math/Math.h>
#include <openvdb/math/Stats.h>
#include <openvdb/agents/Diagnostics.h>
#include <openvdb/agents/Statistics.h>
#include <openvdb/agents/LevelSetSphere.h>
#include <openvdb/agents/LevelSetUtil.h>

class TestDiagnostics: public CppUnit::TestCase
{
public:
    CPPUNIT_TEST_SUITE(TestDiagnostics);
    CPPUNIT_TEST(testCheck);
    CPPUNIT_TEST(testDiagnose);
    CPPUNIT_TEST(testCheckLevelSet);
    CPPUNIT_TEST(testCheckFogVolume);
    CPPUNIT_TEST(testUniqueInactiveValues);
    CPPUNIT_TEST_SUITE_END();

    void testCheck();
    void testDiagnose();
    void testCheckLevelSet();
    void testCheckFogVolume();
    void testUniqueInactiveValues();
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestDiagnostics);


////////////////////////////////////////

void
TestDiagnostics::testCheck()
{
    const float val = 1.0f;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf1= std::numeric_limits<float>::infinity();
    const openvdb::math::Vec3<float> inf2(val, inf1, val);

    {//test CheckNan
        openvdb::agents::CheckNan<openvdb::FloatGrid> c;
        CPPUNIT_ASSERT(!c(val));
        CPPUNIT_ASSERT( c(nan));
        CPPUNIT_ASSERT( c(nan));
        CPPUNIT_ASSERT(!c(inf1));
        CPPUNIT_ASSERT(!c(inf2));
    }
    {//test CheckInf
        openvdb::agents::CheckInf<openvdb::FloatGrid> c;
        CPPUNIT_ASSERT(!c(val));
        CPPUNIT_ASSERT(!c(nan));
        CPPUNIT_ASSERT(!c(nan));
        CPPUNIT_ASSERT( c(inf1));
        CPPUNIT_ASSERT( c(inf2));
    }
    {//test CheckFinite
        openvdb::agents::CheckFinite<openvdb::FloatGrid> c;
        CPPUNIT_ASSERT(!c(val));
        CPPUNIT_ASSERT( c(nan));
        CPPUNIT_ASSERT( c(nan));
        CPPUNIT_ASSERT( c(inf1));
        CPPUNIT_ASSERT( c(inf2));
    }
    {//test CheckMin
        openvdb::agents::CheckMin<openvdb::FloatGrid> c(0.0f);
        CPPUNIT_ASSERT(!c( 0.5f));
        CPPUNIT_ASSERT(!c( 0.0f));
        CPPUNIT_ASSERT(!c( 1.0f));
        CPPUNIT_ASSERT(!c( 1.1f));
        CPPUNIT_ASSERT( c(-0.1f));
    }
    {//test CheckMax
        openvdb::agents::CheckMax<openvdb::FloatGrid> c(0.0f);
        CPPUNIT_ASSERT( c( 0.5f));
        CPPUNIT_ASSERT(!c( 0.0f));
        CPPUNIT_ASSERT( c( 1.0f));
        CPPUNIT_ASSERT( c( 1.1f));
        CPPUNIT_ASSERT(!c(-0.1f));
    }
    {//test CheckRange
        // first check throw on construction from an invalid range
        CPPUNIT_ASSERT_THROW(openvdb::agents::CheckRange<openvdb::FloatGrid> c(1.0f, 0.0f),
                             openvdb::ValueError);
        openvdb::agents::CheckRange<openvdb::FloatGrid> c(0.0f, 1.0f);
        CPPUNIT_ASSERT(!c(0.5f));
        CPPUNIT_ASSERT(!c(0.0f));
        CPPUNIT_ASSERT(!c(1.0f));
        CPPUNIT_ASSERT( c(1.1f));
        CPPUNIT_ASSERT(c(-0.1f));
    }
}//testCheck

void
TestDiagnostics::testDiagnose()
{
    using namespace openvdb;
    const float val = 1.0f;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    {//empty grid
        FloatGrid grid;
        agents::Diagnose<FloatGrid> d(grid);
        agents::CheckNan<FloatGrid> c;
        std::string str = d.check(c);
        //std::cerr << "Empty grid:\n" << str;
        CPPUNIT_ASSERT_EQUAL(std::string(), str);
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {//non-empty grid
        FloatGrid grid;
        grid.tree().setValue(Coord(-1,3,6), val);
        agents::Diagnose<FloatGrid> d(grid);
        agents::CheckNan<FloatGrid> c;
        std::string str = d.check(c);
        //std::cerr << "Non-Empty grid:\n" << str;
        CPPUNIT_ASSERT_EQUAL(std::string(), str);
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {//nan grid
        FloatGrid grid;
        grid.tree().setValue(Coord(-1,3,6), nan);
        agents::Diagnose<FloatGrid> d(grid);
        agents::CheckNan<FloatGrid> c;
        std::string str = d.check(c);
        //std::cerr << "NaN grid:\n" << str;
        CPPUNIT_ASSERT(!str.empty());
        CPPUNIT_ASSERT_EQUAL(1, int(d.failureCount()));
    }

    {//nan and infinite grid
        FloatGrid grid;
        grid.tree().setValue(Coord(-1,3,6), nan);
        grid.tree().setValue(Coord(10,30,60), inf);
        agents::Diagnose<FloatGrid> d(grid);
        agents::CheckFinite<FloatGrid> c;
        std::string str = d.check(c);
        //std::cerr << "Not Finite grid:\n" << str;
        CPPUNIT_ASSERT(!str.empty());
        CPPUNIT_ASSERT_EQUAL(2, int(d.failureCount()));
    }
    {//out-of-range grid
        FloatGrid grid(10.0f);
        grid.tree().setValue(Coord(-1,3,6), 1.0f);
        grid.tree().setValue(Coord(10,30,60), 1.5);
        grid.tree().fill(math::CoordBBox::createCube(math::Coord(0),8), 20.0f, true);
        agents::Diagnose<FloatGrid> d(grid);
        agents::CheckRange<FloatGrid> c(0.0f, 1.0f);
        std::string str = d.check(c);
        //std::cerr << "out-of-range grid:\n" << str;
        CPPUNIT_ASSERT(!str.empty());
        CPPUNIT_ASSERT_EQUAL(3, int(d.failureCount()));
    }

    const float radius = 4.3f;
    const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
    const float voxelSize = 0.1f, width = 2.0f, gamma=voxelSize*width;

    FloatGrid::Ptr gridSphere =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, width);

    //gridSphere->print(std::cerr, 2);

    {// Check min/max of active values
        math::Extrema ex = agents::extrema(gridSphere->cbeginValueOn());
        //std::cerr << "Min = " << ex.min() << " max = " << ex.max() << std::endl;
        CPPUNIT_ASSERT(ex.min() > -voxelSize*width);
        CPPUNIT_ASSERT(ex.max() <  voxelSize*width);

    }
    {// Check min/max of all values
        math::Extrema ex = agents::extrema(gridSphere->cbeginValueAll());
        //std::cerr << "Min = " << ex.min() << " max = " << ex.max() << std::endl;
        CPPUNIT_ASSERT(ex.min() >= -voxelSize*width);
        CPPUNIT_ASSERT(ex.max() <=  voxelSize*width);

    }
    {// check range of all values in a sphere w/o mask
        agents::CheckRange<FloatGrid, true, true, FloatGrid::ValueAllCIter> c(-gamma, gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "Values out of range:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check range of on values in a sphere w/o mask
        agents::CheckRange<FloatGrid, true, true, FloatGrid::ValueOnCIter> c(-gamma, gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "Values out of range:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check range of off tiles in a sphere w/o mask
        agents::CheckRange<FloatGrid, true, true, FloatGrid::ValueOffCIter> c(-gamma, gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        {// check off tile iterator
            FloatGrid::ValueOffCIter i(gridSphere->tree());
            i.setMaxDepth(FloatGrid::ValueOffCIter::LEAF_DEPTH - 1);
            for (; i; ++i) CPPUNIT_ASSERT( math::Abs(*i) <= gamma);
        }
        std::string str = d.check(c);
        //std::cerr << "Values out of range:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check range of sphere w/o mask
        agents::CheckRange<FloatGrid> c(0.0f, gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "Values out of range:\n" << str;
        CPPUNIT_ASSERT(!str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT(d.failureCount() <  gridSphere->activeVoxelCount());
    }
    {// check range of sphere w mask
        agents::CheckRange<FloatGrid> c(0.0f, gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c, true);
        //std::cerr << "Values out of range:\n" << str;
        CPPUNIT_ASSERT(!str.empty());
        CPPUNIT_ASSERT_EQUAL(d.valueCount(), d.valueCount());
        CPPUNIT_ASSERT(d.failureCount() <  gridSphere->activeVoxelCount());
    }
    {// check min of sphere w/o mask
        agents::CheckMin<FloatGrid> c(-gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "Min values:\n" << str;
        CPPUNIT_ASSERT_EQUAL(std::string(), str);
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check max of sphere w/o mask
        agents::CheckMax<FloatGrid> c(gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "MAX values:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check norm of gradient of sphere w/o mask
        agents::CheckEikonal<FloatGrid> c(*gridSphere, 0.97f, 1.03f);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c, false, true, false, false);
        //std::cerr << "NormGrad:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check norm of gradient of sphere w/o mask
        agents::CheckNormGrad<FloatGrid> c(*gridSphere, 0.75f, 1.25f);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c, false, true, false, false);
        //std::cerr << "NormGrad:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
    {// check inactive values
        agents::CheckMagnitude<FloatGrid, FloatGrid::ValueOffCIter> c(gamma);
        agents::Diagnose<FloatGrid> d(*gridSphere);
        std::string str = d.check(c);
        //std::cerr << "Magnitude:\n" << str;
        CPPUNIT_ASSERT(str.empty());
        CPPUNIT_ASSERT_EQUAL(0, int(d.valueCount()));
        CPPUNIT_ASSERT_EQUAL(0, int(d.failureCount()));
    }
}// testDiagnose

void
TestDiagnostics::testCheckLevelSet()
{
    using namespace openvdb;
    const float radius = 4.3f;
    const Vec3f center(15.8f, 13.2f, 16.7f);
    const float voxelSize = 0.1f, width = LEVEL_SET_HALF_WIDTH;

    FloatGrid::Ptr grid =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, width);

    //agents::CheckLevelSet<FloatGrid> c(*grid);
    //std::string str = c.check();
    std::string str = agents::checkLevelSet(*grid);
    CPPUNIT_ASSERT(str.empty());
    //std::cerr << "\n" << str << std::endl;

    grid->tree().setValue(Coord(0,0,0), voxelSize*(width+0.5f));
    //str = c.check();
    str = agents::checkLevelSet(*grid);
    CPPUNIT_ASSERT(!str.empty());
    //std::cerr << "\n" << str << std::endl;

    //str = c.check(6);
    str = agents::checkLevelSet(*grid, 6);
    CPPUNIT_ASSERT(str.empty());

}// testCheckLevelSet

void
TestDiagnostics::testCheckFogVolume()
{
    using namespace openvdb;
    const float radius = 4.3f;
    const Vec3f center(15.8f, 13.2f, 16.7f);
    const float voxelSize = 0.1f, width = LEVEL_SET_HALF_WIDTH;

    FloatGrid::Ptr grid =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, width);
    agents::sdfToFogVolume(*grid);

    //agents::CheckFogVolume<FloatGrid> c(*grid);
    //std::string str = c.check();
    std::string str = agents::checkFogVolume(*grid);
    CPPUNIT_ASSERT(str.empty());
    //std::cerr << "\n" << str << std::endl;

    grid->tree().setValue(Coord(0,0,0), 1.5f);
    //str = c.check();
    str = agents::checkFogVolume(*grid);
    CPPUNIT_ASSERT(!str.empty());
    //std::cerr << "\n" << str << std::endl;

    str = agents::checkFogVolume(*grid, 5);
    //str = c.check(5);
    CPPUNIT_ASSERT(str.empty());

}// testCheckFogVolume

void
TestDiagnostics::testUniqueInactiveValues()
{
    openvdb::FloatGrid grid;

    grid.tree().setValueOff(openvdb::Coord(0,0,0), -1);
    grid.tree().setValueOff(openvdb::Coord(0,0,1), -2);
    grid.tree().setValueOff(openvdb::Coord(0,1,0), -3);
    grid.tree().setValue(openvdb::Coord(1,0,0),  1);

    std::vector<float> values;

    CPPUNIT_ASSERT(openvdb::agents::uniqueInactiveValues(grid, values, 4));

    CPPUNIT_ASSERT_EQUAL(4, int(values.size()));

    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[0], -3.0f));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[1], -2.0f));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[2], -1.0f));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[3], 0.0f));


    // test with level set sphere
    const float radius = 4.3f;
    const openvdb::Vec3f center(15.8f, 13.2f, 16.7f);
    const float voxelSize = 0.5f, width = 2.0f;

    openvdb::FloatGrid::Ptr gridSphere =
        openvdb::agents::createLevelSetSphere<openvdb::FloatGrid>(radius, center, voxelSize, width);

    CPPUNIT_ASSERT(openvdb::agents::uniqueInactiveValues(*gridSphere.get(), values, 2));

    CPPUNIT_ASSERT_EQUAL(2, int(values.size()));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[0], -voxelSize * width));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[1],  voxelSize * width));

    // test with fog volume
    openvdb::agents::sdfToFogVolume(*gridSphere);

    CPPUNIT_ASSERT(openvdb::agents::uniqueInactiveValues(*gridSphere.get(), values, 1));

    CPPUNIT_ASSERT_EQUAL(1, int(values.size()));
    CPPUNIT_ASSERT(openvdb::math::isApproxEqual(values[0], 0.0f));
}
