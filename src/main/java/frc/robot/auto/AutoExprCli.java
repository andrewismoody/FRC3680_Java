package frc.robot.auto;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;

public final class AutoExprCli {
    private AutoExprCli() {}

    // File logger: append-only log in system temp dir. Null if initialization failed.
    private static final PrintWriter LOG;
    static {
        PrintWriter w = null;
        try {
            File f = new File(System.getProperty("java.io.tmpdir"), "autoexprcli.log");
            // append, UTF-8
            FileOutputStream fos = new FileOutputStream(f, true);
            w = new PrintWriter(new OutputStreamWriter(fos, StandardCharsets.UTF_8), true);
            w.println("=== autoexprcli started: " + java.time.Instant.now() + " ===");
        } catch (Throwable t) {
            // if we can't open a log, leave LOG null
            w = null;
        }
        LOG = w;
    }

    private static void logToFile(String s) {
        if (LOG == null) return;
        try {
            LOG.println(s);
            LOG.flush();
        } catch (Throwable ignore) {}
    }

    private static Double evalExprWithParams(JsonNode root) {
        String expr = root.has("expr") && !root.get("expr").isNull() ? root.get("expr").asText() : null;

        // build params map preserving types: Number -> Double, Text -> String, Array -> List<Object>
        Map<String, Object> paramsRaw = new HashMap<>();
        if (root.has("params") && root.get("params").isObject()) {
            JsonNode pnode = root.get("params");
            Iterator<Map.Entry<String, JsonNode>> it = pnode.fields();
            while (it.hasNext()) {
                Map.Entry<String, JsonNode> e = it.next();
                JsonNode v = e.getValue();
                if (v.isNumber()) paramsRaw.put(e.getKey(), v.doubleValue());
                else if (v.isTextual()) paramsRaw.put(e.getKey(), v.asText());
                else if (v.isArray()) {
                    List<Object> list = new ArrayList<>();
                    for (JsonNode a : v) {
                        if (a.isNumber()) list.add(a.doubleValue());
                        else if (a.isTextual()) list.add(a.asText());
                        else if (a.isNull()) list.add(null);
                        // ignore nested objects
                    }
                    paramsRaw.put(e.getKey(), list);
                }
            }
        }

        // Delegate full evaluation (including recursive param resolution) to AutoExpr.evalWithParams
        return AutoExpr.evalWithParams(expr, paramsRaw);
    }

    // Single-shot mode (backward compatible).
    private static void singleShot(ObjectMapper mapper, String input) throws IOException {
        JsonNode root = mapper.readTree(input);
        Double result = evalExprWithParams(root);
        ObjectNode out = mapper.createObjectNode();
        if (result == null) out.putNull("value");
        else out.put("value", result.doubleValue());
        String jsonOut = mapper.writeValueAsString(out);
        System.out.println(jsonOut);
        logToFile("SINGLE: " + jsonOut);
    }

    // Server mode: read newline-delimited JSON requests, write newline-delimited JSON responses.
    private static void serverMode(ObjectMapper mapper) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in, java.nio.charset.StandardCharsets.UTF_8));
        String line;
        while ((line = reader.readLine()) != null) {
            line = line.trim();
            if (line.isEmpty()) continue;
            try {
                JsonNode root = mapper.readTree(line);
                Double result = evalExprWithParams(root);
                ObjectNode out = mapper.createObjectNode();
                if (result == null) out.putNull("value");
                else out.put("value", result.doubleValue());
                String jsonOut = mapper.writeValueAsString(out);
                System.out.println(jsonOut);
                logToFile("SERVER: in=" + line + " out=" + jsonOut);
                // flush stdout for the caller to receive immediately
                System.out.flush();
            } catch (Throwable t) {
                // On error respond with {"value":null} and continue
                ObjectNode out = mapper.createObjectNode();
                out.putNull("value");
                String jsonOut = mapper.writeValueAsString(out);
                System.out.println(jsonOut);
                logToFile("SERVER ERROR: in=" + line + " ex=" + t.toString());
                if (LOG != null) {
                    t.printStackTrace(LOG);
                    LOG.flush();
                }
                System.out.flush();
            }
        }
    }

    public static void main(String[] args) {
        ObjectMapper mapper = new ObjectMapper();
        try {
            boolean server = false;
            for (var a : args) if ("--server".equals(a)) server = true;

            if (server) {
                serverMode(mapper);
                return;
            }

            // Backward compatible single-shot behavior: read all stdin and exit
            String input = new String(System.in.readAllBytes(), java.nio.charset.StandardCharsets.UTF_8).trim();
            if (!input.isEmpty()) singleShot(mapper, input);
        } catch (IOException ex) {
            logToFile("FATAL IO: " + ex.toString());
            ex.printStackTrace(System.err);
            if (LOG != null) { ex.printStackTrace(LOG); LOG.flush(); }
            System.exit(1);
        } catch (Throwable t) {
            logToFile("FATAL: " + t.toString());
            t.printStackTrace(System.err);
            if (LOG != null) { t.printStackTrace(LOG); LOG.flush(); }
            System.exit(1);
        }
    }
}
