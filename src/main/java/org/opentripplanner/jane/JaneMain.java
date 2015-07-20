package org.opentripplanner.jane;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph.LoadLevel;
import org.opentripplanner.routing.impl.InputStreamGraphSource;
import org.opentripplanner.routing.services.GraphService;
import org.opentripplanner.standalone.OTPMain;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.beust.jcommander.JCommander;
import com.beust.jcommander.ParameterException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.vividsolutions.jts.geom.Coordinate;

/**
 * Added functionalities to incorporate place data in our project collection
 */
public class JaneMain {

	private static final Logger LOG = LoggerFactory.getLogger(JaneMain.class);

	public static final String OTP_CONFIG_FILENAME = "otp-config.json";

	public static void main(String[] args) {

		/* Parse and validate command line parameters. */
		JaneCommandLineParameters params = new JaneCommandLineParameters();
		try {
			JCommander jc = new JCommander(params, args);
			if (params.help) {
				System.out.println("Use --dump to dump all edges contained in graph.");
				System.exit(0);
			}
			params.infer();
		} catch (ParameterException pex) {
			LOG.error("Parameter error: {}", pex.getMessage());
			System.exit(1);
		}

		if (!params.dump) {
			LOG.info("Nothing to do. Use --dump to dump all edges contained in graph.");
			System.exit(-1);
		}

		if (params.routerIds == null || params.routerIds.size() != 1) {
			LOG.info("Use --router to indicate which router you want to work with, one at a time.");
			System.exit(-1);
		}

		LOG.info("Attempting to dump edges from routerIds {}", params.routerIds);
		LOG.info("Graph files will be sought in paths relative to {}", params.graphDirectory);
		for (String routerId : params.routerIds) {
			if (!GraphService.routerIdLegal(routerId)) {
				LOG.error("routerId '{}' contains characters other than alphanumeric, underscore, and dash.", routerId);
				continue;
			}
			ObjectMapper mapper = new ObjectMapper();
			InputStreamGraphSource graphSource = InputStreamGraphSource.newFileGraphSource(routerId,
					new File(params.graphDirectory, routerId), LoadLevel.FULL);
			LOG.info("Retriving router '{}'", routerId);
			graphSource.reload(true, false);
			if (graphSource.getRouter() == null || graphSource.getRouter().graph == null) {
				LOG.warn("Can't obtain graph from router ID '{}'.", routerId);
				continue;
			}
			try (FileOutputStream fop = new FileOutputStream(
					new File(params.graphDirectory, routerId + "/edges.json"))) {
				Map<Integer, JaneEdge> map = new HashMap<>();
				for (Edge e : graphSource.getRouter().graph.getEdges()) {
					if (e.getGeometry() == null)
						continue;
					JaneEdge newEdge = new JaneEdge(e);
					// uncomment the following block to check for duplicated
					// edges
					/*
					 * if (map.containsKey(newEdge.getId())) {
					 * System.out.println(e);
					 * System.out.println(map.get(newEdge.getId())); break; }
					 */
					map.put(newEdge.getId(), newEdge);
				}
				// System.out.println(mapper.writeValueAsString(map));
				mapper.writeValue(fop, map);
				// Map<String, JaneEdge> mapObject =
				// mapper.readValue(mapper.writeValueAsString(map), new
				// TypeReference<Map<String, JaneEdge>>() {});
			} catch (IOException e) {
				LOG.error("IOError when dumping edges in graph from router ID '{}'.", routerId);
				// TODO Auto-generated catch block
				e.printStackTrace();
				continue;
			}
		}
	}

}
