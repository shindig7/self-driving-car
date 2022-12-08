SIGHT_DIST = 6.0

struct Point
  x::Float64
  y::Float64
end

struct Line
  origin::Point
  endpoint::Point
end

mutable struct Car
  position::Point
  velo::Float64
  angle::Float64
  brain::Vector{Float64}
  function Car(position::Point, velo::Float64, angle::Float64)
    brain = [rand() for _ in 1:6]
    return new(position, velo, angle, brain)
  end
end

struct Map
  barriers::Vector{Line}
  goals::Vector{Line}
end

function distance(point1::Point, point2::Point)
  sqrt((point2.x - point1.x)^2 + (point2.y - point1.y)^2)
end

function get_intersection(line1::Line, line2::Line)
  # https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
  a1 = line1.endpoint.y - line1.origin.y
  b1 = line1.origin.x - line1.endpoint.x
  c1 = a1*line1.origin.x + b1*line1.origin.y

  a2 = line2.endpoint.y - line2.origin.y
  b2 = line2.origin.x - line2.endpoint.x
  c2 = a2*line2.origin.x + b2*line2.origin.y

  determinant = a1*b2 - b1*a2
  if (determinant == 0)
    return nothing
  end
  x = (b2*c1 - b1*c2) / determinant
  y = (a1*c2 - a2*c1) / determinant
  return Point(x, y)
end

function does_intersect(line1::Line, line2::Line)
  intersect = get_intersection(line1, line2)
  if isnothing(intersect)
    return false
  end

  return intersect.x < max(line1.origin.x, line1.endpoint.x) && intersect.x > min(line1.origin.x, line1.endpoint.x)

  """
  xvals = [line1.origin.x, line1.endpoint.x, line2.origin.x, line2.endpoint.x]
  yvals = [line1.origin.y, line1.endpoint.y, line2.origin.y, line2.endpoint.y]

  xrange = min(xvals...):max(xvals...)
  yrange = min(yvals...):max(yvals...)
  return intersect.x in xrange && intersect.y in yrange
  return intersect.x < max(xvals...) && intersect.x > min(xvals...) && intersect.y < max(yvals...) && intersect.y > min(yvals...)
  """
end

function accelerate(car::Car, delta_v::Float64)
  car.velo += delta_v
end

function turn(car::Car, delta_angle::Float64)
  car.angle += delta_angle
end

function get_endpoint(origin::Point, angle::Float64, length::Float64)
  pos_y = origin.y + (sind(angle) * length)
  pos_x = origin.x + (cosd(angle) * length)
  return Point(pos_x, pos_y)
end

function move!(car::Car)
  car.position = get_endpoint(car.position, car.angle, car.velo)
end


function get_sightlines(car::Car)
  angles = car.angle .+ [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
  endpoints = [get_endpoint(car.position, angle, SIGHT_DIST) for angle in angles]
  return [Line(car.position, ep) for ep in endpoints]
end

function sight(car::Car, wmap::Map)
  sightlines = get_sightlines(car)
  out = []
  for sl in sightlines
    dists = []
    for barrier in wmap.barriers
      if does_intersect(sl, barrier)
        push!(dists, distance(car.position, get_intersection(sl, barrier)))
      end
    end
    if length(dists) == 0
      push!(out, SIGHT_DIST)
    else
      push!(out, min(dists...))
    end
  end
  return out
end
        
function breed(parent1::Car, parent2::Car)
  # Assume that the parent weight vectors are stored in the variables
  # "parent1" and "parent2", and that they have the same length
  
  # Set the crossover probability
  p_crossover = 0.8
  
  # Create the child weight vector
  child = zeros(length(parent1.brain))
  
  # Crossover the parent weight vectors with probability p_crossover
  if rand() < p_crossover
      # Crossover the weight vectors by taking a weighted average of
      # the weights from the two parent vectors, with the weights
      # being drawn from a uniform distribution
      for i in 1:length(parent1.brain)
          child[i] = (parent1.brain[i] * rand() + parent2.brain[i] * rand()) / 2
      end
  else
      # If crossover does not occur, the child is a copy of one of
      # the parent vectors (chosen at random)
      if rand() < 0.5
          child = copy(parent1.brain)
      else
          child = copy(parent2.brain)
      end
  end
  return child
end

