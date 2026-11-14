package com.thesis.garby.ui.glass

import android.view.ViewTreeObserver
import androidx.annotation.DrawableRes
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.WindowInsetsSides
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.only
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawing
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.statusBars
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.foundation.layout.windowInsetsTopHeight
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.ripple
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawWithCache
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.heading
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import androidx.compose.ui.window.DialogWindowProvider
import com.thesis.garby.R
import com.thesis.garby.performance.LocalPerformanceProfile
import com.thesis.garby.ui.rememberResourceBitmap
import com.thesis.garby.ui.hideSystemBars
import com.thesis.garby.ui.theme.GarbyColors

object GlassShapes {
    val Hero = RoundedCornerShape(32.dp)
    val Card = RoundedCornerShape(26.dp)
    val Inner = RoundedCornerShape(18.dp)
    val Pill = RoundedCornerShape(percent = 50)
}

/**
 * Glass thickness. Text and numbers sit on [Regular] or [Thick] so contrast does not depend on
 * what happens to be behind them. Smoked fills avoid live blur/render-target costs on all GPUs.
 */
enum class GlassTier(
    internal val fill: Color,
    internal val sheenTop: Float,
    internal val sheenMid: Float
) {
    /** Chips and secondary chrome. */
    Clear(Color(0xC71B0B0F), 0.10f, 0.02f),

    /** Cards that carry text and numbers. */
    Regular(Color(0xE61B0B0F), 0.12f, 0.03f),

    /** Bars, docks and dense text. */
    Thick(Color(0xF0160709), 0.10f, 0.02f)
}

/** Light catches the top-left edge of the glass. */
val GlassBorderBrush: Brush = Brush.linearGradient(
    0f to Color.White.copy(alpha = 0.45f),
    0.45f to Color.White.copy(alpha = 0.10f),
    1f to Color.White.copy(alpha = 0.22f)
)

/** Static glass fill: no off-screen blur layers or GPU capability assumptions. */
fun Modifier.glassFill(tier: GlassTier): Modifier = background(tier.fill)

/** The sheen and optional coloured glow drawn over the frosted fill. */
fun Modifier.glassSheen(tier: GlassTier, glow: Color? = null): Modifier = drawWithCache {
    val sheen = Brush.linearGradient(
        0f to Color.White.copy(alpha = tier.sheenTop),
        0.5f to Color.White.copy(alpha = tier.sheenMid),
        1f to Color.Transparent,
        start = Offset.Zero,
        end = Offset(size.width, size.height)
    )
    val glowBrush = glow?.let {
        Brush.radialGradient(
            colors = listOf(it.copy(alpha = 0.32f), it.copy(alpha = 0f)),
            center = Offset(size.width * 0.12f, 0f),
            radius = size.maxDimension * 0.8f
        )
    }
    onDrawBehind {
        drawRect(sheen)
        glowBrush?.let { drawRect(it) }
    }
}

/** A frosted glass card. */
@Composable
fun GlassPanel(
    modifier: Modifier = Modifier,
    shape: Shape = GlassShapes.Card,
    tier: GlassTier = GlassTier.Regular,
    glow: Color? = null,
    contentPadding: PaddingValues = PaddingValues(18.dp),
    onClick: (() -> Unit)? = null,
    onClickLabel: String? = null,
    verticalArrangement: Arrangement.Vertical = Arrangement.Top,
    content: @Composable ColumnScope.() -> Unit
) {
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    val click = if (onClick != null) {
        Modifier.clickable(onClickLabel = onClickLabel, role = Role.Button, onClick = onClick)
    } else {
        Modifier
    }
    Column(
        modifier = modifier
            .clip(shape)
            .glassFill(tier)
            .glassSheen(tier, glow.takeUnless { reducedEffects })
            .border(1.dp, GlassBorderBrush, shape)
            .then(click)
            .padding(contentPadding),
        verticalArrangement = verticalArrangement,
        content = content
    )
}

enum class Backdrop(@param:DrawableRes val image: Int, val dim: Float) {
    /** Dashboard and return screens: the vivid marble that makes the glass read. */
    Vivid(R.drawable.app_background, 0.30f),

    /** Start screen. */
    Deep(R.drawable.darker_app_background, 0.20f)
}

/** Sampled marble artwork with cached shading behind the smoked glass. */
@Composable
fun GlassBackdrop(backdrop: Backdrop, modifier: Modifier = Modifier) {
    val profile = LocalPerformanceProfile.current
    val bitmap by rememberResourceBitmap(backdrop.image, profile.backdropMaxDimensionPx)
    Box(modifier.background(GarbyColors.Night)) {
        bitmap?.let {
            Image(
                bitmap = it,
                contentDescription = null,
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize()
            )
        }
        Box(Modifier.fillMaxSize().drawWithCache {
            val dim = Brush.verticalGradient(
                0f to GarbyColors.Night.copy(alpha = backdrop.dim + 0.35f),
                0.35f to GarbyColors.Night.copy(alpha = backdrop.dim),
                0.7f to GarbyColors.Night.copy(alpha = backdrop.dim + 0.05f),
                1f to GarbyColors.Night.copy(alpha = backdrop.dim + 0.45f)
            )
            val ember = if (profile.reducedEffects) null else Brush.radialGradient(
                colors = listOf(GarbyColors.EmberGlow.copy(alpha = 0.28f), Color.Transparent),
                center = Offset(size.width * 0.15f, size.height * 0.22f),
                radius = size.minDimension * 0.75f
            )
            val rose = if (profile.reducedEffects) null else Brush.radialGradient(
                colors = listOf(GarbyColors.Rose.copy(alpha = 0.14f), Color.Transparent),
                center = Offset(size.width * 0.9f, size.height * 0.68f),
                radius = size.minDimension * 0.8f
            )
            onDrawBehind {
                drawRect(dim)
                ember?.let { drawRect(it) }
                rose?.let { drawRect(it) }
            }
        })
    }
}

/** Screen frame with a static backdrop and floating smoked-glass bars. */
@Composable
fun GlassScaffold(
    modifier: Modifier = Modifier,
    backdrop: Backdrop = Backdrop.Vivid,
    topBar: @Composable () -> Unit = {},
    bottomBar: @Composable () -> Unit = {},
    content: @Composable (PaddingValues) -> Unit
) {
    Box(modifier.fillMaxSize().background(GarbyColors.Night)) {
        GlassBackdrop(backdrop = backdrop, modifier = Modifier.fillMaxSize())
        Scaffold(
            containerColor = Color.Transparent,
            contentColor = GarbyColors.TextPrimary,
            topBar = topBar,
            bottomBar = bottomBar,
            contentWindowInsets = WindowInsets.safeDrawing
        ) { innerPadding ->
            Box(Modifier.fillMaxSize()) {
                content(innerPadding)
            }
        }
        // Keeps the status-bar icons legible over content scrolling beneath them.
        Box(
            Modifier
                .fillMaxWidth()
                .windowInsetsTopHeight(WindowInsets.statusBars)
                .background(
                    Brush.verticalGradient(
                        listOf(GarbyColors.Night.copy(alpha = 0.75f), Color.Transparent)
                    )
                )
        )
    }
}

/** A floating frosted bar under the status bar. */
@Composable
fun GlassTopBar(
    title: String,
    modifier: Modifier = Modifier,
    subtitle: String? = null,
    subtitleColor: Color = GarbyColors.TextMuted,
    navigationIcon: (@Composable () -> Unit)? = null,
    actions: @Composable RowScope.() -> Unit = {}
) {
    val shape = RoundedCornerShape(24.dp)
    Box(
        modifier
            .fillMaxWidth()
            .windowInsetsPadding(WindowInsets.safeDrawing.only(WindowInsetsSides.Top + WindowInsetsSides.Horizontal))
            .padding(horizontal = 12.dp, vertical = 8.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier
                .fillMaxWidth()
                .heightIn(min = 64.dp)
                .clip(shape)
                .glassFill(GlassTier.Thick)
                .glassSheen(GlassTier.Thick)
                .border(1.dp, GlassBorderBrush, shape)
                .padding(horizontal = 8.dp, vertical = 6.dp)
        ) {
            if (navigationIcon != null) {
                navigationIcon()
            } else {
                Spacer(Modifier.width(8.dp))
            }
            Column(
                Modifier
                    .weight(1f)
                    .padding(horizontal = 8.dp)
            ) {
                Text(
                    text = title,
                    style = MaterialTheme.typography.titleLarge,
                    color = GarbyColors.TextPrimary,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.semantics { heading() }
                )
                if (subtitle != null) {
                    Text(
                        text = subtitle,
                        style = MaterialTheme.typography.labelMedium,
                        color = subtitleColor,
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis
                    )
                }
            }
            Row(verticalAlignment = Alignment.CenterVertically, content = actions)
        }
    }
}

/** A floating frosted dock above the navigation bar, for the screen's primary action. */
@Composable
fun GlassDock(
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit
) {
    val shape = RoundedCornerShape(30.dp)
    Box(
        modifier
            .fillMaxWidth()
            .windowInsetsPadding(WindowInsets.safeDrawing.only(WindowInsetsSides.Bottom + WindowInsetsSides.Horizontal))
            .padding(start = 12.dp, end = 12.dp, top = 4.dp, bottom = 10.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .clip(shape)
                .glassFill(GlassTier.Thick)
                .glassSheen(GlassTier.Thick)
                .border(1.dp, GlassBorderBrush, shape)
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
            content = content
        )
    }
}

/** A 48 dp round glass button around an icon. */
@Composable
fun GlassIconButton(
    icon: ImageVector,
    contentDescription: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
    tint: Color = GarbyColors.TextPrimary
) {
    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .size(48.dp)
            .clip(CircleShape)
            .background(Color.White.copy(alpha = 0.08f))
            .border(1.dp, Color.White.copy(alpha = 0.16f), CircleShape)
            .clickable(
                enabled = enabled,
                role = Role.Button,
                onClickLabel = contentDescription,
                onClick = onClick
            )
    ) {
        Icon(
            imageVector = icon,
            contentDescription = contentDescription,
            tint = if (enabled) tint else tint.copy(alpha = 0.4f),
            modifier = Modifier.size(22.dp)
        )
    }
}

enum class GlassButtonStyle { Primary, Secondary }

/** A pill-shaped glass button. The primary style glows in the brand ember. */
@Composable
fun GlassButton(
    text: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    style: GlassButtonStyle = GlassButtonStyle.Primary,
    enabled: Boolean = true,
    loading: Boolean = false,
    icon: ImageVector? = null,
    accent: Color = GarbyColors.Ember
) {
    val shape = GlassShapes.Pill
    val interaction = remember { MutableInteractionSource() }
    val pressed by interaction.collectIsPressedAsState()
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    val scale = if (reducedEffects) rememberUpdatedState(1f) else animateFloatAsState(
        targetValue = if (pressed && enabled && !loading) 0.97f else 1f,
        animationSpec = tween(120),
        label = "glassButtonPress"
    )
    val active = enabled && !loading
    val primary = style == GlassButtonStyle.Primary
    val fill: Brush = when {
        !enabled -> SolidColor(Color.White.copy(alpha = 0.06f))
        primary -> Brush.horizontalGradient(
            listOf(accent.copy(alpha = 0.95f), GarbyColors.Crimson.copy(alpha = 0.95f))
        )
        else -> SolidColor(Color.White.copy(alpha = 0.10f))
    }
    val contentColor = if (enabled) GarbyColors.TextPrimary else GarbyColors.TextMuted
    val glow = if (primary && enabled && !reducedEffects) {
        Modifier.shadow(18.dp, shape, ambientColor = accent, spotColor = accent)
    } else {
        Modifier
    }

    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .then(if (reducedEffects) Modifier else Modifier.graphicsLayer {
                scaleX = scale.value
                scaleY = scale.value
            })
            .then(glow)
            .heightIn(min = 56.dp)
            .clip(shape)
            .background(fill)
            .glassSheen(GlassTier.Clear)
            .border(
                width = 1.dp,
                brush = if (enabled) GlassBorderBrush else SolidColor(Color.White.copy(alpha = 0.10f)),
                shape = shape
            )
            .clickable(
                interactionSource = interaction,
                indication = ripple(color = Color.White),
                enabled = active,
                role = Role.Button,
                onClick = onClick
            )
            .padding(horizontal = 24.dp, vertical = 14.dp)
    ) {
        if (loading) {
            CircularProgressIndicator(
                color = contentColor,
                strokeWidth = 2.5.dp,
                modifier = Modifier.size(24.dp)
            )
        } else {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.Center
            ) {
                if (icon != null) {
                    Icon(icon, contentDescription = null, tint = contentColor, modifier = Modifier.size(20.dp))
                    Spacer(Modifier.width(10.dp))
                }
                Text(
                    text = text,
                    style = MaterialTheme.typography.labelLarge,
                    color = contentColor,
                    textAlign = TextAlign.Center
                )
            }
        }
    }
}

/** An icon inside a tinted glass circle. */
@Composable
fun IconOrb(icon: ImageVector, color: Color, modifier: Modifier = Modifier, size: Dp = 40.dp) {
    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .size(size)
            .clip(CircleShape)
            .background(Brush.linearGradient(listOf(color.copy(alpha = 0.32f), color.copy(alpha = 0.10f))))
            .border(1.dp, color.copy(alpha = 0.45f), CircleShape)
    ) {
        Icon(icon, contentDescription = null, tint = color, modifier = Modifier.size(size * 0.55f))
    }
}

/** A small status capsule: coloured dot or icon plus a word, so colour is never the only cue. */
@Composable
fun StatusPill(
    text: String,
    color: Color,
    modifier: Modifier = Modifier,
    icon: ImageVector? = null
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = modifier
            .clip(GlassShapes.Pill)
            .background(color.copy(alpha = 0.10f))
            .border(1.dp, color.copy(alpha = 0.28f), GlassShapes.Pill)
            .padding(horizontal = 9.dp, vertical = 4.dp)
    ) {
        if (icon != null) {
            Icon(icon, contentDescription = null, tint = color, modifier = Modifier.size(14.dp))
        } else {
            Box(
                modifier = Modifier.size(12.dp).background(color.copy(alpha = 0.18f), CircleShape),
                contentAlignment = Alignment.Center
            ) {
                StatusDot(color = color, size = 6.dp)
            }
        }
        Spacer(Modifier.width(6.dp))
        Text(
            text = text,
            style = MaterialTheme.typography.labelMedium,
            color = color,
            letterSpacing = 0.1.sp,
            maxLines = 1,
            overflow = TextOverflow.Ellipsis
        )
    }
}

@Composable
fun StatusDot(color: Color, modifier: Modifier = Modifier, size: Dp = 8.dp) {
    Box(
        modifier
            .size(size)
            .clip(CircleShape)
            .background(color)
    )
}

/** A category divider with a stable category accent and a separate, labelled live indicator. */
@Composable
fun SectionHeader(
    title: String,
    icon: ImageVector,
    accent: Color,
    modifier: Modifier = Modifier,
    caption: String? = null,
    trailing: (@Composable () -> Unit)? = null
) {
    val shape = RoundedCornerShape(20.dp)
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(top = 14.dp, bottom = 2.dp)
            .clip(shape)
            .background(GarbyColors.NightRaised.copy(alpha = 0.96f))
            .background(Brush.horizontalGradient(listOf(accent.copy(alpha = 0.09f), Color.Transparent)))
            .border(1.dp, GarbyColors.GlassStrokeFaint, shape)
            .drawWithCache {
                val rule = Brush.horizontalGradient(
                    listOf(accent.copy(alpha = 0.65f), accent.copy(alpha = 0.12f), Color.Transparent)
                )
                onDrawBehind {
                    drawLine(rule, Offset(18.dp.toPx(), size.height), Offset(size.width, size.height), 2.dp.toPx())
                }
            }
            .semantics(mergeDescendants = true) { heading() }
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        // The status can use the full width when larger text moves it to the next line.
        FlowRow(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                modifier = Modifier.align(Alignment.CenterVertically)
            ) {
                Box(
                    modifier = Modifier
                        .size(38.dp)
                        .background(accent.copy(alpha = 0.10f), RoundedCornerShape(12.dp))
                        .border(1.dp, accent.copy(alpha = 0.24f), RoundedCornerShape(12.dp)),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(icon, contentDescription = null, tint = accent, modifier = Modifier.size(20.dp))
                }
                Text(
                    text = title,
                    style = MaterialTheme.typography.titleMedium,
                    color = GarbyColors.TextPrimary
                )
            }
            if (trailing != null) {
                Box(Modifier.align(Alignment.CenterVertically)) { trailing() }
            }
        }
        if (caption != null) {
            Text(caption, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextMuted)
        }
    }
}

/**
 * A modal dialog drawn as thick smoked glass over the system dim.
 */
@Composable
fun GlassDialog(
    onDismissRequest: () -> Unit,
    content: @Composable ColumnScope.() -> Unit
) {
    Dialog(
        onDismissRequest = onDismissRequest,
        properties = DialogProperties(usePlatformDefaultWidth = false, decorFitsSystemWindows = false)
    ) {
        // Dialogs own a separate window; apply the same policy while a confirmation is open.
        val dialogView = LocalView.current
        DisposableEffect(dialogView) {
            val dialogWindow = (dialogView.parent as? DialogWindowProvider)?.window
            val observer = dialogView.viewTreeObserver
            val listener = ViewTreeObserver.OnWindowFocusChangeListener { focused ->
                if (focused) dialogWindow?.hideSystemBars()
            }
            dialogWindow?.hideSystemBars()
            observer.addOnWindowFocusChangeListener(listener)
            onDispose {
                if (observer.isAlive) observer.removeOnWindowFocusChangeListener(listener)
            }
        }
        GlassPanel(
            tier = GlassTier.Thick,
            shape = GlassShapes.Hero,
            glow = GarbyColors.Ember,
            contentPadding = PaddingValues(24.dp),
            modifier = Modifier
                .windowInsetsPadding(WindowInsets.safeDrawing)
                .padding(24.dp)
                .fillMaxWidth()
                .widthCap(),
            content = content
        )
    }
}

/** Keeps dialogs and centred panels readable on tablets. */
fun Modifier.widthCap(max: Dp = 520.dp): Modifier = widthIn(max = max)
