# coding=utf-8
import numpy as np
from comptests import comptest, run_module_tests, comptest_fails
from duckietown_visualodo.algo.utils import second_largest
import duckietown_visualodo




@comptest
def check_util():
    a = [1,2,3,4]
    assert(second_largest(a)==3)


@comptest
def check_is_rotation_matrix():
    M = np.array([[1,0,0],[0,1,0],[0,0,1]])
    assert is_rotation_matrix(M)


if __name__ == '__main__':
    run_module_tests()
