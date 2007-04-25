function model = read_osimFile(filename, ignore_ground_body)

if nargin < 1
    error('read_osimFile: filename not specified');
elseif nargin < 2
    ignore_ground_body = false;
end

model.bodies = [];

xDoc = xmlread(filename);
allListItems = xDoc.getElementsByTagName('SimmBody');
for i=0:allListItems.getLength-1
    thisListItem = allListItems.item(i);

    body_name = char(thisListItem.getAttribute('name'));

    if ignore_ground_body & strcmp(body_name,'ground') 
        continue;
    end

    nodes = thisListItem.getElementsByTagName('mass');
    body_mass = inf;
    for j=0:nodes.getLength-1
        body_mass = sscanf(char(nodes.item(j).getFirstChild.getData),'%f');
    end

    %childNode = thisListItem.getFirstChild;
    %while ~isempty(childNode)
    %    if childNode.getNodeType == childNode.ELEMENT_NODE & strcmp(char(childNode.getTagName),'mass')
    %        body_mass = sscanf(char(childNode.getFirstChild.getData),'%f');
    %    end
    %    childNode = childNode.getNextSibling;
    %end

    body_index = length(model.bodies)+1;
    model.bodies(body_index).name = body_name;
    model.bodies(body_index).mass = body_mass;
end
