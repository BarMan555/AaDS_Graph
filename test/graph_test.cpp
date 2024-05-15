#include <gtest/gtest.h>
#include "../include/set.h"

using namespace SetSpace;

TEST(set_test, insert)
{
	Set set;
	set.insert(0);
	
	ASSERT_TRUE(set.contains(0));
}

TEST(set_test, contains)
{
	Set set = { 0 };
	bool in_set = set.contains(0);

	ASSERT_TRUE(in_set);
}

TEST(set_test, erase)
{
	Set set = { 0, 1, 2 };
	set.erase(0);
	bool in_set = set.contains(0);

	ASSERT_FALSE(in_set);
}

TEST(set_test, equel)
{
	Set set1 = { 0, 1, 2 };
	Set set2 = { 0, 1, 2 };
	ASSERT_TRUE(set1 == set2);
}

TEST(task_test, intersection_normal)
{
	Set set1 = {0, 10, 5, 25, 15};
	Set set2 = {50, 5, 10, 20, 7};
	
	Set result_right = {5, 10};
	Set result_intersection = intersection(set1, set2);
	ASSERT_EQ(result_intersection, result_right);
}

TEST(task_test, intersection_empty)
{
	Set set1, set2;
	Set result_right;
	Set result_intersection = intersection(set1, set2);
	ASSERT_EQ(result_intersection, result_right);
}

TEST(task_test, intersection_empty_one)
{
	Set set1, set2 = {5};
	Set result_right;
	Set result_intersection = intersection(set1, set2);
	ASSERT_EQ(result_intersection, result_right);
}

TEST(task_test, difference_normal)
{
	Set set_d1 = {10, 1, 0, 12, 100};
	Set set_d2 = {10, 2, 0, 15, 10};

	Set result_right = {1, 12, 100};
	Set result_difference = difference(set_d1, set_d2);
	ASSERT_EQ(result_difference, result_right);
}

TEST(task_test, difference_empty_both)
{
	Set set_d1, set_d2, result_right;
	Set result_difference = difference(set_d1, set_d2);
	ASSERT_EQ(result_difference, result_right);
}

TEST(task_test, difference_empty_first)
{
	Set set_d1, set_d2 = {10, 12, 11}, result_right;
	Set result_difference = difference(set_d1, set_d2);
	ASSERT_EQ(result_difference, result_right);
}

TEST(task_test, difference_empty_second)
{
	Set set_d1 = {1, 2, 3}, set_d2;
	Set result_right(set_d1);
	Set result_difference = difference(set_d1, set_d2);
	ASSERT_EQ(result_difference, result_right);
}