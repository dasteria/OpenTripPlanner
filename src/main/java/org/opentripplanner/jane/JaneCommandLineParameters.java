package org.opentripplanner.jane;

import com.beust.jcommander.Parameter;

import org.opentripplanner.standalone.CommandLineParameters;

public class JaneCommandLineParameters extends CommandLineParameters {
	@Parameter(names = { "--dump" }, description = "dump edges into a json file.")
	public boolean dump;
}
