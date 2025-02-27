// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/// @file unittest/TestPotentialFlow.cc

#include <cppunit/extensions/HelperMacros.h>
#include <openvdb/openvdb.h>
#include <openvdb/agents/LevelSetSphere.h>
#include <openvdb/agents/PotentialFlow.h>


class TestPotentialFlow: public CppUnit::TestCase
{
public:
    CPPUNIT_TEST_SUITE(TestPotentialFlow);
    CPPUNIT_TEST(testMask);
    CPPUNIT_TEST(testNeumannVelocities);
    CPPUNIT_TEST(testUniformStream);
    CPPUNIT_TEST(testFlowAroundSphere);
    CPPUNIT_TEST_SUITE_END();
    void testMask();
    void testNeumannVelocities();
    void testUniformStream();
    void testFlowAroundSphere();
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestPotentialFlow);


void
TestPotentialFlow::testMask()
{
    using namespace openvdb;

    const float radius = 1.5f;
    const Vec3f center(0.0f, 0.0f, 0.0f);
    const float voxelSize = 0.25f;
    const float halfWidth = 3.0f;

    FloatGrid::Ptr sphere =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, halfWidth);

    const int dilation = 5;

    MaskGrid::Ptr mask = agents::createPotentialFlowMask(*sphere, dilation);
    MaskGrid::Ptr defaultMask = agents::createPotentialFlowMask(*sphere);
    CPPUNIT_ASSERT(*mask == *defaultMask);

    auto acc = mask->getAccessor();

    // the isosurface of this sphere is at y = 6
    // this mask forms a band dilated outwards from the isosurface by 5 voxels

    CPPUNIT_ASSERT(!acc.isValueOn(Coord(0, 5, 0)));
    CPPUNIT_ASSERT(acc.isValueOn(Coord(0, 6, 0)));
    CPPUNIT_ASSERT(acc.isValueOn(Coord(0, 10, 0)));
    CPPUNIT_ASSERT(!acc.isValueOn(Coord(0, 11, 0)));

    { // error on non-uniform voxel size
        FloatGrid::Ptr nonUniformSphere =
            agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, halfWidth);
        math::Transform::Ptr nonUniformTransform(new math::Transform(
            math::MapBase::Ptr(new math::ScaleMap(Vec3d(0.1, 0.2, 0.3)))));
        nonUniformSphere->setTransform(nonUniformTransform);

        CPPUNIT_ASSERT_THROW(agents::createPotentialFlowMask(*nonUniformSphere, dilation),
            openvdb::ValueError);
    }

    // this is the minimum mask of one voxel either side of the isosurface

    mask = agents::createPotentialFlowMask(*sphere, 2);

    acc = mask->getAccessor();

    CPPUNIT_ASSERT(!acc.isValueOn(Coord(0, 5, 0)));
    CPPUNIT_ASSERT(acc.isValueOn(Coord(0, 6, 0)));
    CPPUNIT_ASSERT(acc.isValueOn(Coord(0, 7, 0)));
    CPPUNIT_ASSERT(!acc.isValueOn(Coord(0, 8, 0)));

    // these should all produce the same masks as the dilation value is clamped

    MaskGrid::Ptr negativeMask = agents::createPotentialFlowMask(*sphere, -1);
    MaskGrid::Ptr zeroMask = agents::createPotentialFlowMask(*sphere, 0);
    MaskGrid::Ptr oneMask = agents::createPotentialFlowMask(*sphere, 1);

    CPPUNIT_ASSERT(*negativeMask == *mask);
    CPPUNIT_ASSERT(*zeroMask == *mask);
    CPPUNIT_ASSERT(*oneMask == *mask);
}


void
TestPotentialFlow::testNeumannVelocities()
{
    using namespace openvdb;

    const float radius = 1.5f;
    const Vec3f center(0.0f, 0.0f, 0.0f);
    const float voxelSize = 0.25f;
    const float halfWidth = 3.0f;

    FloatGrid::Ptr sphere =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, halfWidth);

    MaskGrid::Ptr domain = agents::createPotentialFlowMask(*sphere);

    {
        // test identical potential from a wind velocity supplied through grid or background value

        Vec3d windVelocityValue(0, 0, 10);

        Vec3dTree::Ptr windTree(new Vec3dTree(sphere->tree(), zeroVal<Vec3d>(), TopologyCopy()));
        dilateVoxels(*windTree, 2, agents::NN_FACE_EDGE_VERTEX);
        windTree->voxelizeActiveTiles();

        for (auto leaf = windTree->beginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->beginValueOn(); iter; ++iter) {
                iter.setValue(windVelocityValue);
            }
        }

        Vec3dGrid::Ptr windGrid(Vec3dGrid::create(windTree));
        windGrid->setTransform(sphere->transform().copy());

        auto windPotentialFromGrid = agents::createPotentialFlowNeumannVelocities(
            *sphere, *domain, windGrid, Vec3d(0));

        CPPUNIT_ASSERT_EQUAL(windPotentialFromGrid->transform(), sphere->transform());

        auto windPotentialFromBackground = agents::createPotentialFlowNeumannVelocities(
            *sphere, *domain, Vec3dGrid::Ptr(), windVelocityValue);

        auto accessor = windPotentialFromGrid->getConstAccessor();
        auto accessor2 = windPotentialFromBackground->getConstAccessor();

        CPPUNIT_ASSERT_EQUAL(windPotentialFromGrid->activeVoxelCount(),
            windPotentialFromBackground->activeVoxelCount());

        for (auto leaf = windPotentialFromGrid->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT_EQUAL(accessor.isValueOn(iter.getCoord()),
                    accessor2.isValueOn(iter.getCoord()));
                CPPUNIT_ASSERT_EQUAL(accessor.getValue(iter.getCoord()),
                    accessor2.getValue(iter.getCoord()));
            }
        }

        // test potential from a wind velocity supplied through grid background value

        Vec3dTree::Ptr emptyWindTree(
            new Vec3dTree(sphere->tree(), windVelocityValue, TopologyCopy()));
        Vec3dGrid::Ptr emptyWindGrid(Vec3dGrid::create(emptyWindTree));
        emptyWindGrid->setTransform(sphere->transform().copy());

        auto windPotentialFromGridBackground = agents::createPotentialFlowNeumannVelocities(
            *sphere, *domain, emptyWindGrid, Vec3d(0));

        CPPUNIT_ASSERT_EQUAL(windPotentialFromGridBackground->transform(), sphere->transform());

        accessor = windPotentialFromGridBackground->getConstAccessor();
        accessor2 = windPotentialFromBackground->getConstAccessor();

        CPPUNIT_ASSERT_EQUAL(windPotentialFromGridBackground->activeVoxelCount(),
            windPotentialFromBackground->activeVoxelCount());

        for (auto leaf = windPotentialFromGridBackground->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT_EQUAL(accessor.isValueOn(iter.getCoord()),
                    accessor2.isValueOn(iter.getCoord()));
                CPPUNIT_ASSERT_EQUAL(accessor.getValue(iter.getCoord()),
                    accessor2.getValue(iter.getCoord()));
            }
        }

        // test potential values are double when applying wind velocity
        // through grid and background values

        auto windPotentialFromBoth = agents::createPotentialFlowNeumannVelocities(
            *sphere, *domain, windGrid, windVelocityValue);

        agents::prune(windPotentialFromBoth->tree(), Vec3d(1e-3));
        agents::prune(windPotentialFromBackground->tree(), Vec3d(1e-3));

        accessor = windPotentialFromBoth->getConstAccessor();
        accessor2 = windPotentialFromBackground->getConstAccessor();

        for (auto leaf = windPotentialFromBoth->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT_EQUAL(accessor.isValueOn(iter.getCoord()),
                    accessor2.isValueOn(iter.getCoord()));
                CPPUNIT_ASSERT_EQUAL(accessor.getValue(iter.getCoord()),
                    accessor2.getValue(iter.getCoord()) * 2);
            }
        }

        CPPUNIT_ASSERT(*windPotentialFromBoth == *windPotentialFromBackground);
    }

    Vec3dGrid::Ptr zeroVelocity = Vec3dGrid::create(Vec3d(0));

    { // error if grid is not a levelset
        FloatGrid::Ptr nonLevelSetSphere =
            agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, halfWidth);
        nonLevelSetSphere->setGridClass(GRID_FOG_VOLUME);

        CPPUNIT_ASSERT_THROW(agents::createPotentialFlowNeumannVelocities(
            *nonLevelSetSphere, *domain, zeroVelocity, Vec3d(5)), openvdb::TypeError);
    }

    { // accept double level set grid
        DoubleGrid::Ptr doubleSphere =
            agents::createLevelSetSphere<DoubleGrid>(radius, center, voxelSize, halfWidth);

        CPPUNIT_ASSERT_NO_THROW(agents::createPotentialFlowNeumannVelocities(
            *doubleSphere, *domain, zeroVelocity, Vec3d(5)));
    }

    { // zero boundary velocities and background velocity
        Vec3d zeroVelocityValue(zeroVal<Vec3d>());
        auto neumannVelocities = agents::createPotentialFlowNeumannVelocities(
            *sphere, *domain, zeroVelocity, zeroVelocityValue);
        CPPUNIT_ASSERT_EQUAL(neumannVelocities->activeVoxelCount(), Index64(0));
    }
}


void
TestPotentialFlow::testUniformStream()
{
    // this unit test checks the scalar potential and velocity flow field
    // for a uniform stream which consists of a 100x100x100 cube of
    // neumann voxels with constant velocity (0, 0, 1)

    using namespace openvdb;

    auto transform = math::Transform::createLinearTransform(1.0);

    auto mask = MaskGrid::create(false);
    mask->setTransform(transform);
    auto maskAccessor = mask->getAccessor();

    auto neumann = Vec3dGrid::create(Vec3d(0));
    auto neumannAccessor = neumann->getAccessor();

    for (int i = -50; i < 50; i++) {
        for (int j = -50; j < 50; j++) {
            for (int k = -50; k < 50; k++) {
                Coord ijk(i, j, k);
                maskAccessor.setValueOn(ijk, true);
                neumannAccessor.setValueOn(ijk, Vec3d(0, 0, 1));
            }
        }
    }

    openvdb::math::pcg::State state = math::pcg::terminationDefaults<float>();

    state.iterations = 2000;
    state.absoluteError = 1e-8;

    auto potential = agents::computeScalarPotential(*mask, *neumann, state);

    // check convergence

    CPPUNIT_ASSERT(state.success);
    CPPUNIT_ASSERT(state.iterations > 0 && state.iterations < 1000);
    CPPUNIT_ASSERT(state.absoluteError < 1e-6);

    CPPUNIT_ASSERT_EQUAL(potential->activeVoxelCount(), mask->activeVoxelCount());

    // for uniform flow along the z-axis, the scalar potential should be equal to the z co-ordinate

    for (auto leaf = potential->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
            const double staggeredZ = iter.getCoord().z() + 0.5;
            CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue(), staggeredZ, /*tolerance*/0.1));
        }
    }

    auto flow = agents::computePotentialFlow(*potential, *neumann);

    CPPUNIT_ASSERT_EQUAL(flow->activeVoxelCount(), mask->activeVoxelCount());

    // flow velocity should be equal to the input velocity (0, 0, 1)

    for (auto leaf = flow->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
            CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue().x(), 0.0, /*tolerance*/1e-6));
            CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue().y(), 0.0, /*tolerance*/1e-6));
            CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue().z(), 1.0, /*tolerance*/1e-6));
        }
    }
}


void
TestPotentialFlow::testFlowAroundSphere()
{
    using namespace openvdb;

    const float radius = 1.5f;
    const Vec3f center(0.0f, 0.0f, 0.0f);
    const float voxelSize = 0.25f;
    const float halfWidth = 3.0f;

    const int dilation = 50;

    FloatGrid::Ptr sphere =
        agents::createLevelSetSphere<FloatGrid>(radius, center, voxelSize, halfWidth);

    MaskGrid::Ptr domain = agents::createPotentialFlowMask(*sphere, dilation);

    { // compute potential flow for a global wind velocity around a sphere

        Vec3f windVelocity(0, 0, 1);
        Vec3fGrid::Ptr neumann = agents::createPotentialFlowNeumannVelocities(*sphere,
            *domain, Vec3fGrid::Ptr(), windVelocity);

        openvdb::math::pcg::State state = math::pcg::terminationDefaults<float>();

        state.iterations = 2000;
        state.absoluteError = 1e-8;

        FloatGrid::Ptr potential = agents::computeScalarPotential(*domain, *neumann, state);

        // compute a laplacian of the potential within the domain (excluding neumann voxels)
        // and ensure it evaluates to zero

        auto mask = BoolGrid::create(/*background=*/false);
        mask->setTransform(potential->transform().copy());
        mask->topologyUnion(*potential);

        auto dilatedSphereMask = agents::interiorMask(*sphere);
        agents::dilateActiveValues(dilatedSphereMask->tree(), 1);
        mask->topologyDifference(*dilatedSphereMask);

        FloatGrid::Ptr laplacian = agents::laplacian(*potential, *mask);

        for (auto leaf = laplacian->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue(), 0.0f, /*tolerance*/1e-3f));
            }
        }

        Vec3fGrid::Ptr flowVel = agents::computePotentialFlow(*potential, *neumann);

        // compute the divergence of the flow velocity within the domain
        // (excluding neumann voxels and exterior voxels)
        // and ensure it evaluates to zero

        agents::erodeVoxels(mask->tree(), 2, agents::NN_FACE);

        FloatGrid::Ptr divergence = agents::divergence(*flowVel, *mask);

        for (auto leaf = divergence->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue(), 0.0f, /*tolerance*/0.1f));
            }
        }

        // check the background velocity has been applied correctly

        Vec3fGrid::Ptr flowVelBackground =
            agents::computePotentialFlow(*potential, *neumann, windVelocity);

        CPPUNIT_ASSERT_EQUAL(flowVelBackground->activeVoxelCount(),
            flowVelBackground->activeVoxelCount());

        auto maskAccessor = mask->getConstAccessor();

        auto accessor = flowVel->getConstAccessor();
        auto accessor2 = flowVelBackground->getConstAccessor();

        for (auto leaf = flowVelBackground->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                // ignore values near the neumann boundary
                if (!maskAccessor.isValueOn(iter.getCoord()))    continue;

                const Vec3f value1 = accessor.getValue(iter.getCoord());
                const Vec3f value2 = accessor2.getValue(iter.getCoord()) + windVelocity;

                CPPUNIT_ASSERT(math::isApproxEqual(value1.x(), value2.x(), /*tolerance=*/1e-3f));
                CPPUNIT_ASSERT(math::isApproxEqual(value1.y(), value2.y(), /*tolerance=*/1e-3f));
                CPPUNIT_ASSERT(math::isApproxEqual(value1.z(), value2.z(), /*tolerance=*/1e-3f));
            }
        }
    }

    { // check double-precision solve
        DoubleGrid::Ptr sphereDouble =
            agents::createLevelSetSphere<DoubleGrid>(radius, center, voxelSize, halfWidth);

        Vec3d windVelocity(0, 0, 1);
        Vec3dGrid::Ptr neumann = agents::createPotentialFlowNeumannVelocities(*sphereDouble,
            *domain, Vec3dGrid::Ptr(), windVelocity);

        openvdb::math::pcg::State state = math::pcg::terminationDefaults<float>();

        state.iterations = 2000;
        state.absoluteError = 1e-8;

        DoubleGrid::Ptr potential = agents::computeScalarPotential(*domain, *neumann, state);

        CPPUNIT_ASSERT(potential);

        // compute a laplacian of the potential within the domain (excluding neumann voxels)
        // and ensure it evaluates to zero

        auto mask = BoolGrid::create(/*background=*/false);
        mask->setTransform(potential->transform().copy());
        mask->topologyUnion(*potential);

        auto dilatedSphereMask = agents::interiorMask(*sphereDouble);
        agents::dilateActiveValues(dilatedSphereMask->tree(), 1);
        mask->topologyDifference(*dilatedSphereMask);

        DoubleGrid::Ptr laplacian = agents::laplacian(*potential, *mask);

        for (auto leaf = laplacian->tree().cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                CPPUNIT_ASSERT(math::isApproxEqual(iter.getValue(), 0.0, /*tolerance*/1e-5));
            }
        }

        Vec3dGrid::Ptr flowVel = agents::computePotentialFlow(*potential, *neumann);

        CPPUNIT_ASSERT(flowVel);
    }
}
