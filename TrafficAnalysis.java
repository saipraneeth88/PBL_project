import java.util.*;

// Location class to represent points on the map
class Location {
    String name;
    double latitude;
    double longitude;

    public Location(String name, double latitude, double longitude) {
        this.name = name;
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

// Route class to represent a path from source to destination
class Route {
    Location source;
    Location destination;
    List<Location> path;
    double distance;

    public Route(Location source, Location destination, List<Location> path, double distance) {
        this.source = source;
        this.destination = destination;
        this.path = path;
        this.distance = distance;
    }
}

// PointOfInterest class for places like fuel stations and restaurants
class PointOfInterest {
    String type;
    Location location;

    public PointOfInterest(String type, Location location) {
        this.type = type;
        this.location = location;
    }
}

// Graph class implementing Dijkstra's algorithm for shortest path
class Graph {
    private final Map<Location, List<Location>> adjacencyMap = new HashMap<>();

    public void addEdge(Location source, Location destination, double distance) {
        adjacencyMap.putIfAbsent(source, new ArrayList<>());
        adjacencyMap.get(source).add(destination);
    }

    public Route findShortestRoute(Location source, Location destination) {
        Map<Location, Double> distances = new HashMap<>();
        Map<Location, Location> previousLocations = new HashMap<>();
        PriorityQueue<Location> priorityQueue = new PriorityQueue<>(Comparator.comparing(distances::get));

        for (Location location : adjacencyMap.keySet()) {
            if (location.equals(source)) {
                distances.put(location, 0.0);
            } else {
                distances.put(location, Double.MAX_VALUE);
            }
            priorityQueue.add(location);
        }

        while (!priorityQueue.isEmpty()) {
            Location current = priorityQueue.poll();
            if (current.equals(destination)) {
                break;
            }

            for (Location neighbor : adjacencyMap.getOrDefault(current, new ArrayList<>())) {
                double newDist = distances.get(current) + calculateDistance(current, neighbor);
                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    previousLocations.put(neighbor, current);
                    priorityQueue.add(neighbor);
                }
            }
        }

        List<Location> path = new ArrayList<>();
        for (Location at = destination; at != null; at = previousLocations.get(at)) {
            path.add(at);
        }
        Collections.reverse(path);
        return new Route(source, destination, path, distances.get(destination));
    }

    private double calculateDistance(Location loc1, Location loc2) {
        // Example function to calculate distance
        return Math.sqrt(Math.pow(loc1.latitude - loc2.latitude, 2) + Math.pow(loc1.longitude - loc2.longitude, 2));
    }
}

// RoutePlanner class to manage points of interest along the route
class RoutePlanner {
    List<PointOfInterest> pointsOfInterest = new ArrayList<>();

    public void addPointOfInterest(PointOfInterest poi) {
        pointsOfInterest.add(poi);
    }

    public void displayPointsOfInterest(Route route) {
        for (PointOfInterest poi : pointsOfInterest) {
            if (isAlongRoute(route, poi)) {
                System.out.println(poi.type + " at " + poi.location.name);
            }
        }
    }

    private boolean isAlongRoute(Route route, PointOfInterest poi) {
        // Example logic to check if POI is along the route
        return route.path.contains(poi.location);
    }
}

// Navigator class to handle navigation and rerouting
class Navigator {
    Graph graph;
    RoutePlanner routePlanner;

    public Navigator(Graph graph, RoutePlanner routePlanner) {
        this.graph = graph;
        this.routePlanner = routePlanner;
    }

    public void navigate(Location source, Location destination) {
        Route route = graph.findShortestRoute(source, destination);
        System.out.println("Navigating from " + source.name + " to " + destination.name);
        routePlanner.displayPointsOfInterest(route);

        // Example rerouting
        if (needsRerouting()) {
            Location newDestination = new Location("New Destination", 0, 0);
            Route newRoute = graph.findShortestRoute(source, newDestination);
            System.out.println("Rerouted to " + newDestination.name);
            routePlanner.displayPointsOfInterest(newRoute);
        }
    }

    private boolean needsRerouting() {
        // Example logic for rerouting
        return new Random().nextBoolean();
    }
}

// Main class to run the Traffic Analysis program
public class TrafficAnalysis {
    public static void main(String[] args) {
        Graph graph = new Graph();
        RoutePlanner routePlanner = new RoutePlanner();
        Navigator navigator = new Navigator(graph, routePlanner);

        Location locA = new Location("A", 0, 0);
        Location locB = new Location("B", 1, 1);
        Location locC = new Location("C", 2, 2);

        graph.addEdge(locA, locB, 1.0);
        graph.addEdge(locB, locC, 1.0);

        PointOfInterest poi1 = new PointOfInterest("Fuel Station", locB);
        routePlanner.addPointOfInterest(poi1);

        navigator.navigate(locA, locC);
    }
}
