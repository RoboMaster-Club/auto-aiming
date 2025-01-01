#include "ValidityFilter.hpp"
#include "gtest/gtest.h"

class ValidityFilterTest : public ::testing::Test
{
protected:
    ValidityFilter *filter;

    void SetUp() override
    {
        filter = new ValidityFilter();
    }

    void TearDown() override
    {
        delete filter;
    }
};