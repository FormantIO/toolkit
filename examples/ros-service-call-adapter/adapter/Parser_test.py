
# from ..adapter.input_parser import parse

from input_parser import parse
import rospy
from ast import Assert, literal_eval

def numeric_assert(val: str):
    val_calc = parse(val)
    val_actual = literal_eval(val)
    if val_calc != val_actual:
        raise Exception("err")


def assert_equal(val1, val2):
    if val1 != val2:
        raise Exception("err")


def assert_fail(val: str):
    try:
        parse(val)
        raise Exception(f"Expecting failure on {val}, but it was parsed")
    except SyntaxError:
        pass


numeric_assert("-1")
numeric_assert("-0.5")
numeric_assert("0.5")
numeric_assert(".5")
numeric_assert("-.1")
numeric_assert("1.123")
numeric_assert("1.")
numeric_assert("-1.")

assert_equal(False, parse("False"))
assert_equal(True, parse("True"))

assert_equal([1, 2, 3, 4], parse("[1,2,3,4]"))
assert_equal({1, 2, 3, 4}, parse("{1,2,3,4}"))
assert_equal((1, 2, 3, 4), parse("(1,2,3,4)"))
assert_equal([1, 2, 3, [1, 2]], parse("[1,2,3,[1,2]]"))
assert_equal([(1, 2, 3), {1, 2}, 3, 4], parse("[(1,  2  ,3),{1,2    },3,4]"))


assert_fail("[")
assert_fail("(")
assert_fail("{")
assert_fail("]")
assert_fail(")")
assert_fail("}")

assert_fail("-")
assert_fail("..1")
assert_fail("1.233.1")
assert_fail("123-1")

assert_fail("Fals")
assert_fail("Ture")

assert_fail("FalseTrue")

assert_equal(parse("t(1,2)"), rospy.Time(1, 2))
assert_equal(parse("d(5,6)"), rospy.Duration(5,6))
assert_equal(parse("h( 100, t(5,6), 'frame_id!' )"), rospy.Header(seq=100, stamp=rospy.Time(5,6), frame_id='frame_id!'))

assert_fail("d('test', 3)")

assert_fail("h( 100, t(5,6), 3)")
assert_fail("h( 'test', t(5,6), '3')")
assert_fail("h( 100, d(5,6), 3)")
assert_fail("h[(] 100, d(5,6), 3]")
assert_fail("h( 100, d(5,6))")
assert_fail("d(1,2,3)")

# TODO test escape sequences
