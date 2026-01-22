package frc.robot.auto;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.fasterxml.jackson.databind.JsonNode;

import java.util.Map;
import java.util.List;
import java.util.Set;
import java.util.HashSet;

public final class AutoExpr {
    private AutoExpr() {}

    public static Double eval(
            String expr,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver) {

        return eval(expr, scalarResolver, vec2Resolver, null);
    }

    public static Double eval(
            String expr,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver,
            BiFunction<String, Character, Double> vec3Resolver) {

        if (expr == null) return null;
        expr = expr.trim();
        if (expr.isEmpty()) return null;

        try {
            Parser p = new Parser(expr, scalarResolver, vec2Resolver, vec3Resolver);
            Double v = p.parseExpression();
            p.skipWs();
            if (!p.eof()) return null; // trailing junk
            return v;
        } catch (RuntimeException ex) {
            return null;
        }
    }

    public static Double evalNode(
            JsonNode node,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver) {

        return evalNode(node, scalarResolver, vec2Resolver, null);
    }

    public static Double evalNode(
            JsonNode node,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver,
            BiFunction<String, Character, Double> vec3Resolver) {

        if (node == null || node.isNull()) return null;
        if (node.isNumber()) return node.doubleValue();
        if (node.isTextual()) return eval(node.asText(), scalarResolver, vec2Resolver, vec3Resolver);
        return null;
    }

    public static Double evalWithParams(String expr, Map<String, Object> params) {
        if (expr == null) return null;
        try {
            Parser p = new Parser(expr, params);
            Double v = p.parse();
            if (p.hasRemaining()) return null;
            return v;
        } catch (Throwable t) {
            // On any error return null as caller expects
            return null;
        }
    }

    // Replace the Parser with a unified implementation that supports both resolver-mode and params-mode.
    private static final class Parser {
        private final String s;
        private final int n;
        private int i = 0;

        private final Function<String, Double> scalarResolver;
        private final BiFunction<String, Character, Double> vec2Resolver;
        private final BiFunction<String, Character, Double> vec3Resolver;
        private final Map<String, Object> params;
        private final Set<String> resolving = new HashSet<>();

        Parser(String s,
               Function<String, Double> scalarResolver,
               BiFunction<String, Character, Double> vec2Resolver,
               BiFunction<String, Character, Double> vec3Resolver) {
            this.s = s;
            this.n = s.length();
            this.scalarResolver = scalarResolver;
            this.vec2Resolver = vec2Resolver;
            this.vec3Resolver = vec3Resolver;
            this.params = null;
        }

        Parser(String s, Map<String, Object> params) {
            this.s = s;
            this.n = s.length();
            this.params = params;
            this.scalarResolver = null;
            this.vec2Resolver = null;
            this.vec3Resolver = null;
        }

        // Caller convenience (used by eval(...))
        Double parseExpression() {
            Double v = parseTerm();
            if (v == null) return null;

            while (true) {
                skipWs();
                if (eat('+')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v + r;
                } else if (eat('-')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v - r;
                } else break;
            }
            return v;
        }

        // Caller convenience (used by evalWithParams(...))
        Double parse() {
            skipWS();
            Double v = parseExpr();
            skipWS();
            return v;
        }

        boolean eof() { return i >= n; }

        boolean hasRemaining() {
            skipWS();
            return i < n;
        }

        // ---- shared parsing pieces ----
        // The classic expression grammar is implemented in two equivalent sets of methods
        // so both parseExpression() and parse() semantics work. They share core helpers.

        // parseExpr used by parse()
        private Double parseExpr() {
            Double v = parseTerm();
            if (v == null) return null;
            while (true) {
                skipWS();
                if (match('+')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v + r;
                } else if (match('-')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v - r;
                } else break;
            }
            return v;
        }

        private Double parseTerm() {
            Double v = parseUnary();
            if (v == null) return null;
            while (true) {
                skipWs();
                if (eat('*')) {
                    Double r = parseUnary();
                    if (r == null) return null;
                    v = v * r;
                } else if (eat('/')) {
                    Double r = parseUnary();
                    if (r == null) return null;
                    if (r == 0.0) return null;
                    v = v / r;
                } else break;
            }
            return v;
        }

        private Double parseUnary() {
            if (eat('+')) return parseUnary();
            if (eat('-')) {
                Double v = parseUnary();
                return (v == null) ? null : -v;
            }
            return parsePrimary();
        }

        private Double parsePrimary() {
            skipWs();
            if (eat('(')) {
                Double v = parseExpression();
                if (v == null) return null;
                if (!eat(')')) return null;
                return v;
            }

            char c = peekNonWs();
            if (c == '$') return parseVar();
            if (isNumberStart(c)) return parseNumber();
            // identifier without leading $ might appear in params-mode (we support that in parseFactor below)
            return null;
        }

        // Older small-parser style for evalWithParams recursion (keeps compatibility)
        private Double parseFactor() {
            skipWS();
            if (match('+')) return parseFactor();
            if (match('-')) {
                Double v = parseFactor();
                return v == null ? null : -v;
            }
            if (match('(')) {
                Double v = parseExpr();
                if (v == null) return null;
                skipWS();
                if (!match(')')) return null;
                return v;
            }
            if (peekIsDigit() || peek() == '.') {
                return parseNumber();
            }
            if (peekIsIdentStart()) {
                String id = parseIdentifier();
                return resolveIdentifier(id);
            }
            return null;
        }

        // parseNumber usable by both modes
        private Double parseNumber() {
            skipWs();
            int start = i;
            boolean sawDot = false;
            while (i < n) {
                char c = s.charAt(i);
                if (Character.isDigit(c)) i++;
                else if (c == '.' && !sawDot) { sawDot = true; i++; }
                else break;
            }
            // optional exponent
            if (i < n && (s.charAt(i) == 'e' || s.charAt(i) == 'E')) {
                int epos = i;
                i++;
                if (i < n && (s.charAt(i) == '+' || s.charAt(i) == '-')) i++;
                int expStart = i;
                while (i < n && Character.isDigit(s.charAt(i))) i++;
                if (i == expStart) i = epos; // rollback if exponent invalid
            }
            if (i == start) return null;
            try {
                return Double.valueOf(s.substring(start, i));
            } catch (NumberFormatException ex) {
                return null;
            }
        }

        // parseVar handles $name and optional .X/.Y/.Z component access and delegates to
        // vec3/vec2 resolvers when present, or to params (lists) if provided.
        private Double parseVar() {
            if (!eat('$')) return null;
            skipWs();
            int start = i;
            if (i >= n) return null;
            // accept letters/digits/underscore
            while (i < n) {
                char c = s.charAt(i);
                if (Character.isLetterOrDigit(c) || c == '_') i++;
                else break;
            }
            if (i == start) return null;
            String name = s.substring(start, i);

            skipWs();
            if (eat('.')) {
                skipWs();
                if (i >= n) return null;
                char comp = Character.toUpperCase(s.charAt(i));
                if (comp != 'X' && comp != 'Y' && comp != 'Z') return null;
                i++;

                // prefer explicit vec3/vec2 resolvers when available
                if (vec3Resolver != null) {
                    Double v3 = vec3Resolver.apply(name, comp);
                    if (v3 != null) return v3;
                }
                if (vec2Resolver != null) {
                    Double v2 = vec2Resolver.apply(name, comp);
                    if (v2 != null) return v2;
                }

                // fallback to params list if present
                if (params != null && params.containsKey(name)) {
                    Object val = params.get(name);
                    if (val instanceof List) {
                        List<?> list = (List<?>) val;
                        int idx = (comp == 'X') ? 0 : (comp == 'Y') ? 1 : 2;
                        if (idx < list.size()) {
                            Object elem = list.get(idx);
                            if (elem instanceof Number) return ((Number) elem).doubleValue();
                            if (elem instanceof String) {
                                // evaluate recursively if string expression
                                Double d = AutoExpr.evalWithParams((String) elem, params);
                                return d;
                            }
                        }
                    }
                }
                return null;
            }

            // no component: scalar resolution
            if (scalarResolver != null) {
                return scalarResolver.apply(name);
            }
            if (params != null) {
                return resolveIdentifier(name);
            }
            return null;
        }

        private String parseIdentifier() {
            int start = i;
            // first char is ident start
            i++;
            while (i < n && (Character.isLetterOrDigit(s.charAt(i)) || s.charAt(i) == '_' || s.charAt(i) == '.')) i++;
            return s.substring(start, i);
        }

        private Double resolveIdentifier(String id) {
            if (params == null || !params.containsKey(id)) return null;
            if (resolving.contains(id)) return null; // cycle
            Object val = params.get(id);
            if (val == null) return null;
            if (val instanceof Number) return ((Number) val).doubleValue();
            if (val instanceof String) {
                resolving.add(id);
                try {
                    return AutoExpr.evalWithParams((String) val, params);
                } finally {
                    resolving.remove(id);
                }
            }
            // other types unsupported for scalar resolution
            return null;
        }

        // ---- small helpers ----
        void skipWs() {
            while (i < n) {
                char c = s.charAt(i);
                if (c == ' ' || c == '\t' || c == '\r' || c == '\n') i++;
                else break;
            }
        }

        private void skipWS() { skipWs(); } // keep previous name variants

        private boolean eat(char c) {
            skipWs();
            if (i < n && s.charAt(i) == c) { i++; return true; }
            return false;
        }

        private boolean match(char c) {
            if (i < n && s.charAt(i) == c) { i++; return true; }
            return false;
        }

        private char peek() {
            return i < n ? s.charAt(i) : '\0';
        }

        private char peekNonWs() {
            int j = i;
            while (j < n && Character.isWhitespace(s.charAt(j))) j++;
            return (j < n) ? s.charAt(j) : '\0';
        }

        private boolean peekIsDigit() {
            return i < n && Character.isDigit(s.charAt(i));
        }

        private boolean peekIsIdentStart() {
            return i < n && (Character.isLetter(s.charAt(i)) || s.charAt(i) == '_');
        }

        static boolean isNumberStart(char c) {
            return Character.isDigit(c) || c == '.';
        }
    }
}
