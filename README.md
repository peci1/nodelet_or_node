# nodelet\_or\_node

Helper package for easy launch of a nodelet either standalone or in a manager.

This `nodelet_or_node` node allows launching nodelets either by loading them
or by running them standalone. In this, it is similar to `nodelet load` and
`nodelet standalone` from the [nodelet](https://wiki.ros.org/nodelet) package.

However, `nodelet_or_node` makes it easy to differentiate the
load/standalone mode by just providing (or not) the nodelet manager name. This
helps writing nice interfaces which easily allow the user to choose whether the
node should run in a nodelet manager or standalone.

A nodelet loaded into a manager using `nodelet_or_node` can be unloaded using
all standard ways for unloading nodelets.

## Examples

In the following example, if the user supplies a non-empty value for the
`manager` arg, then the nodelet is loaded into this manager. When no value is
given, the nodelet is launched standalone.

**choose.launch**

```xml
<launch>
    <arg name="manager" default="" />
    <node name="test" pkg="nodelet_or_node" type="load" args="my_pkg/Nodelet $(arg manager)">
        <param name="param" value="0" />
    </node>
</launch>
```


**standalone.launch**

```xml
<launch>
    <node name="test" pkg="nodelet_or_node" type="load" args="my_pkg/Nodelet">
        <param name="param" value="0" />
    </node>
</launch>
```