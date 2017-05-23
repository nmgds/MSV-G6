/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#define _CXXTEST_HAVE_STD
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/ErrorPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    CxxTest::ErrorPrinter tmp;
    CxxTest::RealWorldDescription::_worldName = "cxxtest";
    status = CxxTest::Main< CxxTest::ErrorPrinter >( tmp, argc, argv );
    return status;
}
bool suite_OvertakerTest_init = false;
#include "/home/sanja/MSV-G6/code/overtaker/testsuites/OvertakerTestSuite.h"

static OvertakerTest suite_OvertakerTest;

static CxxTest::List Tests_OvertakerTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_OvertakerTest( "OvertakerTestSuite.h", 50, "OvertakerTest", suite_OvertakerTest, Tests_OvertakerTest );

static class TestDescription_suite_OvertakerTest_testOvertakerSuccessfullyCreated : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_OvertakerTest_testOvertakerSuccessfullyCreated() : CxxTest::RealTestDescription( Tests_OvertakerTest, suiteDescription_OvertakerTest, 84, "testOvertakerSuccessfullyCreated" ) {}
 void runTest() { suite_OvertakerTest.testOvertakerSuccessfullyCreated(); }
} testDescription_suite_OvertakerTest_testOvertakerSuccessfullyCreated;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
