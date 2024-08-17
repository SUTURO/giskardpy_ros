import os
import uuid
from collections import defaultdict
from copy import deepcopy
from typing import Any, Type, Optional, Dict, Tuple

import numpy as np
import py_trees
import pydot
import rclpy
from py_trees import common, behaviour, utilities, console, composites, decorators
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.composites import Sequence, Selector, Composite
from py_trees.decorators import FailureIsSuccess, Decorator
from py_trees.display import unicode_symbols, ascii_symbols
from py_trees_ros.trees import BehaviourTree
from rclpy.executors import MultiThreadedExecutor

from giskardpy.god_map import god_map
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.send_result import SendResult
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy_ros.tree.branches.clean_up_control_loop import CleanupControlLoop
from giskardpy_ros.tree.branches.control_loop import ControlLoop
from giskardpy_ros.tree.branches.post_processing import PostProcessing
from giskardpy_ros.tree.branches.prepare_control_loop import PrepareControlLoop
from giskardpy_ros.tree.branches.send_trajectories import ExecuteTraj
from giskardpy_ros.tree.branches.wait_for_goal import WaitForGoal
from giskardpy_ros.tree.composites.async_composite import AsyncBehavior
from giskardpy_ros.tree.composites.better_parallel import Parallel
from giskardpy_ros.tree.control_modes import ControlModes
from giskardpy.middleware import get_middleware
from giskardpy.utils.decorators import toggle_on, toggle_off
from giskardpy.utils.utils import create_path


def behavior_is_instance_of(obj: Any, type_: Type) -> bool:
    return isinstance(obj, type_) or hasattr(obj, 'original') and isinstance(obj.original, type_)


class GiskardBT(BehaviourTree):
    tick_hz: float = 10
    control_mode: ControlModes
    wait_for_goal: WaitForGoal
    prepare_control_loop: PrepareControlLoop
    post_processing: PostProcessing
    cleanup_control_loop: CleanupControlLoop
    control_loop_branch: ControlLoop
    root: Sequence
    execute_traj: ExecuteTraj

    def __init__(self, control_mode: ControlModes):
        GiskardBlackboard().tree = self
        self.control_mode = control_mode
        if control_mode not in ControlModes:
            raise AttributeError(f'Control mode {control_mode} doesn\'t exist.')
        self.root = Sequence('Giskard', memory=True)
        self.wait_for_goal = WaitForGoal()
        self.prepare_control_loop = PrepareControlLoop()
        self.prepare_control_loop_failure_is_success = FailureIsSuccess('ignore failure',
                                                                        self.prepare_control_loop)
        if self.is_closed_loop():
            max_hz = GiskardBlackboard().control_loop_max_hz
        else:
            max_hz = GiskardBlackboard().simulation_max_hz
        self.control_loop_branch = ControlLoop(max_hz=max_hz)
        self.control_loop_branch_failure_is_success = FailureIsSuccess('ignore failure', self.control_loop_branch)
        if self.is_closed_loop():
            self.control_loop_branch.add_closed_loop_behaviors()
        else:
            self.control_loop_branch.add_projection_behaviors()

        self.post_processing = PostProcessing()
        self.post_processing_failure_is_success = FailureIsSuccess('ignore failure', self.post_processing)
        self.cleanup_control_loop = CleanupControlLoop()
        if self.is_open_loop():
            self.execute_traj = ExecuteTraj()
            self.execute_traj_failure_is_success = FailureIsSuccess('ignore failure', self.execute_traj)

        self.root.add_child(self.wait_for_goal)
        self.root.add_child(self.prepare_control_loop_failure_is_success)
        self.root.add_child(self.control_loop_branch_failure_is_success)
        self.root.add_child(self.cleanup_control_loop)
        self.root.add_child(self.post_processing_failure_is_success)
        self.root.add_child(SendResult(GiskardBlackboard().move_action_server))
        super().__init__(self.root, unicode_tree_debug=False)
        self.switch_to_execution()

    def has_started(self) -> bool:
        return self.count > 1

    def is_closed_loop(self):
        return self.control_mode == self.control_mode.close_loop

    def is_standalone(self):
        return self.control_mode == self.control_mode.standalone

    def is_open_loop(self):
        return self.control_mode == self.control_mode.open_loop

    @toggle_on('visualization_mode')
    def turn_on_visualization(self):
        self.wait_for_goal.publish_state.add_visualization_marker_behavior()
        self.control_loop_branch.publish_state.add_visualization_marker_behavior()

    @toggle_off('visualization_mode')
    def turn_off_visualization(self):
        self.wait_for_goal.publish_state.remove_visualization_marker_behavior()
        self.control_loop_branch.publish_state.remove_visualization_marker_behavior()

    @toggle_on('projection_mode')
    def switch_to_projection(self):
        if self.is_open_loop():
            self.root.remove_child(self.execute_traj_failure_is_success)
        elif self.is_closed_loop():
            self.control_loop_branch.switch_to_projection()
        self.cleanup_control_loop.add_reset_world_state()

    @toggle_off('projection_mode')
    def switch_to_execution(self):
        if self.is_open_loop():
            self.root.insert_child(self.execute_traj_failure_is_success, -2)
        elif self.is_closed_loop():
            self.control_loop_branch.switch_to_closed_loop()
        self.cleanup_control_loop.remove_reset_world_state()

    def live(self):
        get_middleware().loginfo('giskard is ready')
        self.tick_tock(period_ms=1000.0)
        rospy.spinner_thread.join()
        self.shutdown()
        rclpy.try_shutdown()
        get_middleware().loginfo('giskard died')

    def stop_spinning(self):
        self.executer.shutdown()

    def kill_all_services(self):
        self.blackboard_exchange.get_blackboard_variables_srv.shutdown()
        self.blackboard_exchange.open_blackboard_watcher_srv.shutdown()
        self.blackboard_exchange.close_blackboard_watcher_srv.shutdown()
        # for value in GiskardBlackboard().tree_nodes.values():
        #     node = value.node
        #     for attribute_name, attribute in vars(node).items():
        #         if isinstance(attribute, rospy.Service):
        #             attribute.shutdown(reason='life is pain')

    def render(self):
        path = god_map.tmp_folder + 'tree'
        create_path(path)
        render_dot_tree(self.root, name=path)


def render_dot_tree(
        root: behaviour.Behaviour,
        visibility_level: common.VisibilityLevel = common.VisibilityLevel.DETAIL,
        collapse_decorators: bool = False,
        name: Optional[str] = None,
        target_directory: Optional[str] = None,
        with_blackboard_variables: bool = False,
        with_qualified_names: bool = False,
) -> Dict[str, str]:
    """
    Render the dot tree to dot, svg, png. files.

    By default, these are saved in the current
    working directory and will be named with the root behaviour name.

    Args:
        root: the root of a tree, or subtree
        visibility_level: collapse subtrees at or under this level
        collapse_decorators: only show the decorator (not the child)
        name: name to use for the created files (defaults to the root behaviour name)
        target_directory: default is to use the current working directory, set this to redirect elsewhere
        with_blackboard_variables: add nodes for the blackboard variables
        with_qualified_names: print the class names of each behaviour in the dot node

    Example:
        Render a simple tree to dot/svg/png file:

        .. graphviz:: dot/sequence.dot

        .. code-block:: python

            root = py_trees.composites.Sequence(name="Sequence", memory=True)
            for job in ["Action 1", "Action 2", "Action 3"]:
                success_after_two = py_trees.behaviours.StatusQueue(
                    name=job,
                    queue=[py_trees.common.Status.RUNNING],
                    eventually = py_trees.common.Status.SUCCESS
                )
                root.add_child(success_after_two)
            py_trees.display.render_dot_tree(root)

    .. tip::

        A good practice is to provide a command line argument for optional rendering of a program so users
        can quickly visualise what tree the program will execute.
    """
    if target_directory is None:
        target_directory = os.getcwd()
    graph = dot_tree(
        root,
        visibility_level,
        collapse_decorators,
        with_blackboard_variables=with_blackboard_variables,
        with_qualified_names=with_qualified_names,
    )
    filename_wo_extension_to_convert = root.name if name is None else name
    filename_wo_extension = utilities.get_valid_filename(
        filename_wo_extension_to_convert
    )
    filenames: Dict[str, str] = {}
    for extension, writer in {
        # "dot": graph.write,
        "png": graph.write_png,
        # "svg": graph.write_svg,
    }.items():
        filename = filename_wo_extension + "." + extension
        pathname = os.path.join(target_directory, filename)
        print("Writing {}".format(pathname))
        writer(pathname)
        filenames[extension] = pathname
    return filenames


time_function_names = ['__init__', 'setup', 'initialise', 'update']


def get_original_node(node: Behaviour) -> Behaviour:
    if hasattr(node, 'original'):
        return node.original
    return node


def dot_tree(
        root: behaviour.Behaviour,
        visibility_level: common.VisibilityLevel = common.VisibilityLevel.DETAIL,
        collapse_decorators: bool = False,
        with_blackboard_variables: bool = False,
        with_qualified_names: bool = False,
) -> pydot.Dot:
    """
    Paint your tree on a pydot graph.

    .. seealso:: :py:func:`render_dot_tree`.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (optional): collapse subtrees at or under this level
        collapse_decorators (optional): only show the decorator (not the child), defaults to False
        with_blackboard_variables (optional): add nodes for the blackboard variables
        with_qualified_names (optional): print the class information for each behaviour in each node, defaults to False

    Returns:
        pydot.Dot: graph

    Examples:
        .. code-block:: python

            # convert the pydot graph to a string object
            print("{}".format(py_trees.display.dot_graph(root).to_string()))
    """

    def get_node_attributes(node: behaviour.Behaviour) -> Tuple[str, str, str]:
        blackbox_font_colours = {
            common.BlackBoxLevel.DETAIL: "dodgerblue",
            common.BlackBoxLevel.COMPONENT: "lawngreen",
            common.BlackBoxLevel.BIG_PICTURE: "white",
        }
        if isinstance(node, composites.Selector):
            attributes = ("octagon", "cyan", "black")  # octagon
        elif isinstance(node, composites.Sequence):
            attributes = ("box", "orange", "black")
        elif isinstance(node, composites.Parallel):
            attributes = ("parallelogram", "gold", "black")
        elif isinstance(node, decorators.Decorator):
            attributes = ("ellipse", "ghostwhite", "black")
        elif isinstance(node, AsyncBehavior):
            attributes = ('house', 'green', 'black')
        else:
            attributes = ("ellipse", "gray", "black")
        try:
            if node.blackbox_level != common.BlackBoxLevel.NOT_A_BLACKBOX:
                attributes = (
                    attributes[0],
                    "gray20",
                    blackbox_font_colours[node.blackbox_level],
                )
        except AttributeError:
            # it's a blackboard client, not a behaviour, just pass
            pass
        return attributes

    def get_node_label(node_name: str, behaviour: behaviour.Behaviour) -> Tuple[str, str]:
        """
        Create a more detailed string (when applicable) to use for the node name.

        This prefixes the node name with additional information about the node type (e.g. with
        or without memory). Useful when debugging.
        """
        # Custom handling of composites provided by this library. Not currently
        # providing a generic mechanism for others to customise visualisations
        # for their derived composites.
        prefix = ""
        policy = ""
        symbols = unicode_symbols if console.has_unicode() else ascii_symbols
        if isinstance(behaviour, Composite):
            try:
                if behaviour.memory:  # type: ignore[attr-defined]
                    prefix += symbols["memory"]  # console.circled_m
            except AttributeError:
                pass
            try:
                if behaviour.policy.synchronise:  # type: ignore[attr-defined]
                    prefix += symbols["synchronised"]  # console.lightning_bolt
            except AttributeError:
                pass
            try:
                policy = behaviour.policy.__class__.__name__  # type: ignore[attr-defined]
            except AttributeError:
                pass
            try:
                indices = [
                    str(behaviour.children.index(child))
                    for child in behaviour.policy.children  # type: ignore[attr-defined]
                ]
                policy += "({})".format(", ".join(sorted(indices)))
            except AttributeError:
                pass
        node_label = f"{prefix} {node_name}" if prefix else node_name
        if policy:
            node_label += f"\n{str(policy)}"
        if with_qualified_names:
            node_label += f"\n({utilities.get_fully_qualified_name(behaviour)})"

        color = 'black'

        # %% add run time stats to proposed dot name
        function_name_padding = 20
        entry_name_padding = 8
        number_padding = function_name_padding - entry_name_padding
        if hasattr(behaviour, '__times'):
            time_dict = behaviour.__times
        else:
            time_dict = {}
        for function_name in time_function_names:
            if function_name in time_dict:
                times = time_dict[function_name]
                average_time = np.average(times)
                std_time = np.std(times)
                total_time = np.sum(times)
                if total_time > 1:
                    color = 'red'
                node_label += f'\n{function_name.ljust(function_name_padding, "-")}' \
                              f'\n{"  #calls".ljust(entry_name_padding)}{f"={len(times)}".ljust(number_padding)}' \
                              f'\n{"  avg".ljust(entry_name_padding)}{f"={average_time:.7f}".ljust(number_padding)}' \
                              f'\n{"  std".ljust(entry_name_padding)}{f"={std_time:.7f}".ljust(number_padding)}' \
                              f'\n{"  max".ljust(entry_name_padding)}{f"={max(times):.7f}".ljust(number_padding)}' \
                              f'\n{"  sum".ljust(entry_name_padding)}{f"={total_time:.7f}".ljust(number_padding)}'
            else:
                node_label += f'\n{function_name.ljust(function_name_padding, "-")}'
        node_label = f'"{node_label}"'
        return node_label, color

    add_children_stats_to_parent(root)

    fontsize = 9
    blackboard_colour = "blue"  # "dimgray"
    graph = pydot.Dot(graph_type="digraph", ordering="out")
    graph.set_name(
        "pastafarianism"
    )  # consider making this unique to the tree sometime, e.g. based on the root name
    # fonts: helvetica, times-bold, arial (times-roman is the default, but this helps some viewers, like kgraphviewer)
    graph.set_graph_defaults(
        fontname="times-roman"
    )  # splines='curved' is buggy on 16.04, but would be nice to have
    graph.set_node_defaults(fontname="times-roman")
    graph.set_edge_defaults(fontname="times-roman")
    (node_shape, node_colour, node_font_colour) = get_node_attributes(root)
    label, color = get_node_label(root.name, root)
    node_font_colour = color
    node_root = pydot.Node(
        name=root.name,
        label=label,
        shape=node_shape,
        style="filled",
        fillcolor=node_colour,
        fontsize=fontsize,
        fontcolor=node_font_colour,
    )
    graph.add_node(node_root)
    behaviour_id_name_map = {root.id: root.name}

    def add_children_and_edges(
            root: behaviour.Behaviour,
            root_node: pydot.Node,
            root_dot_name: str,
            visibility_level: common.VisibilityLevel,
            collapse_decorators: bool,
    ) -> None:
        if isinstance(root, Decorator) and collapse_decorators:
            return
        if visibility_level < root.blackbox_level:
            node_names = []
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c)
                node_name = c.name
                while node_name in behaviour_id_name_map.values():
                    node_name += "*"
                # node_name = f'"{node_names}"'
                behaviour_id_name_map[c.id] = node_name
                # Node attributes can be found on page 5 of
                #    https://graphviz.gitlab.io/_pages/pdf/dot.1.pdf
                # Attributes that may be useful: tooltip, xlabel
                label, node_font_colour = get_node_label(node_name, c)
                node = pydot.Node(
                    name=node_name,
                    label=label,
                    shape=node_shape,
                    style="filled",
                    fillcolor=node_colour,
                    fontsize=fontsize,
                    color=color,
                    fontcolor=node_font_colour,
                )
                node_names.append(node_name)
                graph.add_node(node)
                edge = pydot.Edge(root_dot_name, node_name)
                graph.add_edge(edge)
                if c.children != []:
                    add_children_and_edges(
                        c, node, node_name, visibility_level, collapse_decorators
                    )

    add_children_and_edges(
        root, node_root, root.name, visibility_level, collapse_decorators
    )

    def create_blackboard_client_node(blackboard_client_name: str) -> pydot.Node:
        return pydot.Node(
            name=blackboard_client_name,
            label=blackboard_client_name,
            shape="ellipse",
            style="filled",
            color=blackboard_colour,
            fillcolor="gray",
            fontsize=fontsize - 2,
            fontcolor=blackboard_colour,
        )

    def add_blackboard_nodes(
            blackboard_id_name_map: Dict[uuid.UUID, str]
    ) -> None:
        data = Blackboard.storage
        metadata = Blackboard.metadata
        clients = Blackboard.clients
        # add client (that are not behaviour) nodes
        subgraph = pydot.Subgraph(
            graph_name="Blackboard",
            id="Blackboard",
            label="Blackboard",
            rank="sink",
        )
        for unique_identifier, client_name in clients.items():
            if unique_identifier not in blackboard_id_name_map:
                subgraph.add_node(create_blackboard_client_node(client_name))
        # add key nodes
        for key in Blackboard.keys():
            try:
                value = utilities.truncate(str(data[key]), 20)
                label = key + ": " + "{}".format(value)
            except KeyError:
                label = key + ": " + "-"
            blackboard_node = pydot.Node(
                key,
                label=label,
                shape="box",
                style="filled",
                color=blackboard_colour,
                fillcolor="white",
                fontsize=fontsize - 1,
                fontcolor=blackboard_colour,
                width=0,
                height=0,
                fixedsize=False,  # only big enough to fit text
            )
            subgraph.add_node(blackboard_node)
            for unique_identifier in metadata[key].read:
                try:
                    edge = pydot.Edge(
                        blackboard_node,
                        blackboard_id_name_map[unique_identifier],
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        blackboard_node,
                        clients[unique_identifier],
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
            for unique_identifier in metadata[key].write:
                try:
                    edge = pydot.Edge(
                        blackboard_id_name_map[unique_identifier],
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        clients[unique_identifier],
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
            for unique_identifier in metadata[key].exclusive:
                try:
                    edge = pydot.Edge(
                        blackboard_id_name_map[unique_identifier],
                        blackboard_node,
                        color="deepskyblue",
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        clients[unique_identifier],
                        blackboard_node,
                        color="deepskyblue",
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
        graph.add_subgraph(subgraph)

    if with_blackboard_variables:
        blackboard_id_name_map = {}
        for b in root.iterate():
            for bb in b.blackboards:
                blackboard_id_name_map[bb.id()] = behaviour_id_name_map[b.id]
        add_blackboard_nodes(blackboard_id_name_map)

    return graph


def add_children_stats_to_parent(parent: Composite) -> None:
    if ((hasattr(parent, 'children') and parent.children != [])
            or (hasattr(parent, '_children') and parent._children != [])):
        children = parent.children
        names2 = [c.name for c in children]
        for name, child in zip(names2, children):
            original_child = get_original_node(child)
            if isinstance(original_child, Composite):
                add_children_stats_to_parent(original_child)

            if not hasattr(parent, '__times'):
                setattr(parent, '__times', defaultdict(list))

            if hasattr(original_child, '__times'):
                time_dict = original_child.__times
            else:
                time_dict = {}
            for function_name in time_function_names:
                if function_name in time_dict:
                    if function_name not in parent.__times:
                        parent.__times = deepcopy(time_dict)
                    else:
                        for i, (v1, v2) in enumerate(zip(parent.__times[function_name], time_dict[function_name])):
                            if i > len(parent.__times[function_name]):
                                parent.__times[function_name].append(v2)
                            else:
                                parent.__times[function_name][i] = v1 + v2
