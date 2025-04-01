function path = reconstructPath(node)
    path = [];
    while ~isempty(node)
        path = [node.pos; path];
        node = node.parent;
    end
end