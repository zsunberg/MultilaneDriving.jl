mutable struct Cached{T}
    value::Nullable{T}

    Cached{T}() where T = new(Nullable{T}())
    Cached{T}(val::T) where T = new(Nullable{T}(val))
end

function set!(cv::Cached{T}, val::T) where T
    cv.value = Nullable(val)
    return val
end
iscached(cv::Cached) = !isnull(cv.value)

Base.get(cv::Cached) = get(cv.value)
function Base.get(f::Function, cv::Cached, args...)
    if iscached(cv)
        return get(cv.value)
    else
        return set!(cv, f(args...))
    end
end
