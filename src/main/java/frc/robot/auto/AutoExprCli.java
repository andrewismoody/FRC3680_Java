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
import java.util.function.BiFunction;
import java.util.function.Function;

public final class AutoExprCli {
    private AutoExprCli() {}

    private static Double evalExprWithParams(JsonNode root) {
        String expr = root.has("expr") && !root.get("expr").isNull() ? root.get("expr").asText() : null;

        // build params map (double) using forgiving rules: number, numeric-string, or first numeric element of array
        Map<String, Double> params = new HashMap<>();
        if (root.has("params") && root.get("params").isObject()) {
            JsonNode pnode = root.get("params");
            Iterator<Map.Entry<String, JsonNode>> it = pnode.fields();
            while (it.hasNext()) {
                Map.Entry<String, JsonNode> e = it.next();
                JsonNode v = e.getValue();
                Double val = null;
                if (v.isNumber()) val = v.doubleValue();
                else if (v.isTextual()) {
                    try { val = Double.parseDouble(v.asText()); } catch (NumberFormatException ex) { val = null; }
                } else if (v.isArray()) {
                    for (JsonNode a : v) {
                        if (a.isNumber()) { val = a.doubleValue(); break; }
                        if (a.isTextual()) {
                            try { val = Double.parseDouble(a.asText()); break; } catch (NumberFormatException ex) { /* ignore */ }
                        }
                    }
                }
                if (val != null) params.put(e.getKey(), val);
            }
        }

        Function<String, Double> scalarResolver = name -> params.get(name);
        BiFunction<String, Character, Double> vec2Resolver = (name, comp) -> null;
        BiFunction<String, Character, Double> vec3Resolver = (name, comp) -> null;

        return AutoExpr.eval(expr, scalarResolver, vec2Resolver, vec3Resolver);
    }

    // Single-shot mode (backward compatible).
    private static void singleShot(ObjectMapper mapper, String input) throws IOException {
        JsonNode root = mapper.readTree(input);
        Double result = evalExprWithParams(root);
        ObjectNode out = mapper.createObjectNode();
        if (result == null) out.putNull("value");
        else out.put("value", result.doubleValue());
        System.out.println(mapper.writeValueAsString(out));
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
                System.out.println(mapper.writeValueAsString(out));
                // flush stdout for the caller to receive immediately
                System.out.flush();
            } catch (Throwable t) {
                // On error respond with {"value":null} and continue
                ObjectNode out = mapper.createObjectNode();
                out.putNull("value");
                System.out.println(mapper.writeValueAsString(out));
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
            ex.printStackTrace(System.err);
            System.exit(1);
        } catch (Throwable t) {
            t.printStackTrace(System.err);
            System.exit(1);
        }
    }
}
