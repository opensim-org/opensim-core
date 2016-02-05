#!/usr/bin/env python

import click
import opensim

def print_version(context, param, value):
    if not value or context.resilient_parsing:
        return
    # TODO configure using CMake.
    click.echo("OpenSim v.4.0 built on 24 October 2015 18:36:14.")
    context.exit()

@click.group()
@click.option('--library', '-L', multiple=True, default=None,
        help='Load a plugin to be used in subcommand.')
@click.option('--version', is_flag=True, callback=print_version,
        expose_value=False, is_eager=True,
        help='Print version and build information.')
def cli(library):
    """Toolkit for musculoskeletal modeling and simulation.
    """
    for lib in library:
        click.echo("Loading library '%s'." % library)


@click.command()
@click.argument('setup_file')
def run(setup_file):
    """Run a tool (IK, CMC, ...) from a setup file."""
    obj = opensim.Model.makeObjectFromFile(str(setup_file))
    tool = opensim.AbstractTool.safeDownCast(obj)
    if tool == None:
        tool = opensim.Tool.safeDownCast(obj)
    if tool == None:
        click.echo("Error: '%s' is not a tool setup file." % setup_file)
        return
    tool.run()

@click.command()
@click.argument('class_name')
@click.argument('property_name', required=False)
def property(class_name, property_name):
    """Get description of any OpenSim property."""
    if property_name == None:
        click.echo("Printing names of properties in class '%s'." % class_name)
        opensim.OpenSimObject.PrintPropertyInfo(class_name, "")
    else:
        click.echo("Printing description of %s's '%s'." % (class_name,
            property_name))
        opensim.OpenSimObject.PrintPropertyInfo(class_name, property_name)

@click.command()
@click.argument('name')
def printxml(name):
    """Print a template XML file for a tool/class. NAME can be the name of
    any OpenSim class (e.g., Model), or any one of the following tool names:
    
        scale
        ik
        id
        rra
        cmc
        analyze

    The file is written with the name `default_<class-name>.xml` to the current
    directory.

    Examples:

        Print a template Model.
    
            opensim printxml Model
    
        Print a template CMC tool setup file.
    
            opensim printxml cmc

    """
    if name == 'scale':
        class_name = 'ScaleTool'
    elif name == 'ik':
        class_name = 'InverseKinematicsTool'
    elif name == 'id':
        class_name = 'InverseDynamicsTool'
    elif name == 'rra':
        class_name = 'RRATool'
    elif name == 'cmc':
        class_name = 'CMCTool'
    else:
        class_name = name
        
    obj = opensim.OpenSimObject.getDefaultInstanceOfType(str(class_name))
    file_name = "default_%s.xml" % str(class_name)
    if obj == None:
        click.echo("Error: Class '%s' does not exist. Did you forget to load a "
                "plugin?" % class_name)
        return
    click.echo("Printing '%s'." % file_name)
    obj.printToXML(file_name)

@click.command()
@click.argument('file_path')
@click.argument('out_path')
def update(file_path, out_path):
    """Update XML file. The file is saved to the XML file version used by this
    version of OpenSim. In an OpenSim XML file, the XML file version appears as
    the "Version" attribute the "OpenSimDocument" element.
    
    Current OpenSim version: TODO 

    Current XML file version: TODO
    
    """
    click.echo("Updating '%s' to '%s'." % (file_path, out_path))
    obj = opensim.OpenSimObject.makeObjectFromFile(file_path)
    obj.printToXML(out_path)

#@click.command()
#@click.argument('package')
#def install(package):
#    """Download and install OpenSim package."""
#    click.echo("Searching for package '%s'." % package)
#    click.echo("Downloading package '%s'." % package)
#    click.echo("Installing package to '%s' to /Applications/OpenSim/plugins." %
#            package)

#@click.command()
#@click.argument('model')
#@click.argument('motion', required=False)
#def visualize(model, motion):
#    """View model (and motion) in visualizer."""
#    click.echo("Opening '%s' in visualizer." % model)
#    if motion != None:
#        click.echo("Playing motion from '%s'." % motion)

#@click.command()
#@click.argument('data_path')
#@click.argument('out_path')
#def convert(data_path, out_path):
#    """Convert data file between formats. For example, .mot to .c3d."""
#    click.echo("Loading '%s'." % data_path)
#    click.echo("Printing '%s'." % out_path)

#@click.command()
#@click.argument('data_file')
#def plot(data_file):
#    """Plot from a data file or states trajectory."""
#    click.echo("Opening plotter with '%s'." % data_file)

#@click.command()
#@click.argument('name')
#@click.argument('classes', nargs=-1, required=False)
#def template(name, classes):
#    """Template files for writing C++ plugins. Creates a CMake project with the
#    necessary boilerplate. The source files will have comments telling you what
#    to fill out. List the names of CLASSES that will be a part of your plugin.
#    """
#    click.echo("Creating plugin project in folder '%s'." % name)

#@click.command()
#def workbook():
#    """Operations related to OpenSim data workbooks."""
#    pass

#@click.command()
#def exportviz():
#    """Export visualization to web server."""
#    pass

#@click.command()
#def shell():
#    """Start an interactive OpenSim python interpreter."""
#    from IPython import embed
#    embed()
#    # TODO slow

#cli.add_command(install)
cli.add_command(property)
cli.add_command(update)
cli.add_command(printxml)
cli.add_command(run)
#cli.add_command(visualize)
#cli.add_command(convert)
#cli.add_command(plot)
#cli.add_command(template)
#cli.add_command(workbook)
#cli.add_command(exportviz)
#cli.add_command(shell)

if __name__ == '__main__':
    cli()

# TODO 

#@click.option('--count', default=1, help='Number of greetings.')
#@click.option('--name', prompt='Your name', help='The person to greet.')





