
function out=limit(in,min,max)
out=int16(in);
if in<int16(min)
    out=int16(min);
end
if in>int16(max)
    out=int16(max);
end
end