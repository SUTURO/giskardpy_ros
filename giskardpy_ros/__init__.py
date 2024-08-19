import os

os.environ['ROS_PYTHON_CHECK_FIELDS'] = '1'
os.environ['LINE_PROFILE'] = '0'

# import builtins
#
# try:
#     builtins.profile  # type: ignore
# except AttributeError:
#     # No line profiler, provide a pass-through version
#     def profile(func):
#         return func


    # builtins.profile = profile  # type: ignore
