function neighbors = getNeighbors(node, slice, sliceNum,goalPos)
    neighbors = [];
    currentSlice = findSlice(node.pos, slice, sliceNum);
    
    if currentSlice < sliceNum+1
        nextSlice = currentSlice + 1;
        for i = 1:size(slice(nextSlice).allowedPos, 1)
            neighborPos = slice(nextSlice).allowedPos(i,:);
            neighborNode.pos = neighborPos;
            neighborNode.g = 0;
            neighborNode.h = sqrt(sum((neighborPos - goalPos).^2));
            neighborNode.f = neighborNode.g + neighborNode.h;
            neighborNode.parent = [];
            neighbors = [neighbors; neighborNode];
        end
    end
end

function sliceIdx = findSlice(pos, slice, sliceNum)
    for i = 1:sliceNum+1
        if ismember(pos, slice(i).allowedPos, 'rows')
            sliceIdx = i;
            return;
        end
    end
    sliceIdx = -1;
end